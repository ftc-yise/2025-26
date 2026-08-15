package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "new sensor experiment", group = "Linear OpMode")
public class JULYNewSensorExperiment extends LinearOpMode {

    // ABS Angle Encoder
    private AnalogInput encoder;
    private double MBangle;

    // SparkFun LSM6DSO breakout over I2C
    private I2cDeviceSynchSimple lsm6dsoSimple;
    private I2cDeviceSynchImplOnSimple lsm6dso;

    // Raw gyro / integrated heading
    private static final int LSM6DSO_I2C_ADDR_7BIT = 0x6B;   // change to 0x6A if your board is strapped that way
    private static final int WHO_AM_I_REG = 0x0F;
    private static final int CTRL1_XL = 0x10;
    private static final int CTRL2_G  = 0x11;
    private static final int CTRL3_C  = 0x12;
    private static final int OUTX_L_G = 0x22;                // gyro output registers start here

    private static final double GYRO_SENSITIVITY_DPS_PER_LSB = 0.00875; // 245 dps full-scale
    private double gyroZBiasDps = 0.0;
    private double headingDeg = 0.0;
    private double lastLoopTimeSec = 0.0;

    // Drive motors
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;

    // Timing / telemetry
    private final ElapsedTime runtime = new ElapsedTime();

    // Tunable constants
    private static final double SLOW_SPEED = 0.45;
    private static final double FULL_SPEED = 1.0;
    private static final double DEADBAND = 0.05;

    // State variables
    private double currentSpeed = FULL_SPEED;
    private boolean prevLeftBumper = false;

    @Override
    public void runOpMode() {

        // hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "RightBackDrive");

        encoder = hardwareMap.get(AnalogInput.class, "encoa");

        // IMPORTANT:
        // Your Robot Config should name this I2C device something like "lsm6dso"
        lsm6dsoSimple = hardwareMap.get(I2cDeviceSynchSimple.class, "lsm6dso");

        lsm6dsoSimple.setI2cAddress(I2cAddr.create7bit(0x6B));

        lsm6dso = new I2cDeviceSynchImplOnSimple(lsm6dsoSimple, true);
        lsm6dso.engage();

        configureLsm6dso();

        // Directions - adjust if a motor is reversed on your robot
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Safety: use BRAKE to hold position when joystick released
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();
        lastLoopTimeSec = runtime.seconds();

        // Calibrate gyro Z bias while sitting still
        gyroZBiasDps = calibrateGyroZBias(200);

        while (opModeIsActive()) {

            // ----- SENSORS -----
            MBangle = (encoder.getVoltage() / 3.2) * 360.0;

            double nowSec = runtime.seconds();
            double dt = nowSec - lastLoopTimeSec;
            lastLoopTimeSec = nowSec;

            double gyroZDps = readGyroZDegPerSec();
            double correctedGyroZDps = gyroZDps - gyroZBiasDps;
            headingDeg = normalizeAngle180(headingDeg + (correctedGyroZDps * dt));

            // ----- DRIVE INPUTS -----
            double rawForward = gamepad1.left_stick_y;
            double rawStrafe  = gamepad1.left_stick_x;
            double rawTurn    = gamepad1.right_stick_x;

            double forward = applyDeadband(rawForward, DEADBAND);
            double strafe  = applyDeadband(rawStrafe, DEADBAND);
            double turn    = applyDeadband(rawTurn, DEADBAND);

            // mecanum wheel powers
            double lf = forward - strafe + turn;
            double rf = -forward - strafe - turn;
            double lb = -forward + strafe + turn;
            double rb = forward + strafe - turn;

            // normalize
            double max = Math.max(
                    Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lb), Math.abs(rb))
            );
            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }

            // speed toggle (left bumper edge)
            if (gamepad1.left_bumper && !prevLeftBumper) {
                currentSpeed = (Math.abs(currentSpeed - FULL_SPEED) < 1e-6) ? SLOW_SPEED : FULL_SPEED;
            }
            prevLeftBumper = gamepad1.left_bumper;

            // apply current speed
            leftFrontDrive.setPower(lf * currentSpeed);
            rightFrontDrive.setPower(rf * currentSpeed);
            leftBackDrive.setPower(lb * currentSpeed);
            rightBackDrive.setPower(rb * currentSpeed);

            // ----- TELEMETRY -----
            telemetry.addData("Mode", "TeleOp");
            telemetry.addData("Runtime", "%.1f s", runtime.seconds());
            telemetry.addLine("=== SENSORS ===");
            telemetry.addData("ABS Encoder", "%.2f deg", MBangle);
            telemetry.addData("LSM6DSO gyro Z", "%.3f dps", gyroZDps);
            telemetry.addData("Gyro Z bias", "%.3f dps", gyroZBiasDps);
            telemetry.addData("Heading estimate", "%.2f deg", headingDeg);

            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Speed Mode", (currentSpeed == FULL_SPEED) ? "FULL" : "SLOW");
            telemetry.addData("LF/RF/LB/RB", "%.2f, %.2f, %.2f, %.2f", lf, rf, lb, rb);
            telemetry.update();
        }

        lsm6dso.disengage();
    }

    private double applyDeadband(double v, double deadband) {
        return (Math.abs(v) < deadband) ? 0.0 : v;
    }

    private void configureLsm6dso() {
        telemetry.addLine("Configuring LSM6DSO...");
        telemetry.update();

        lsm6dso.engage();

        // Basic sanity check
        int whoAmI = readRegister(WHO_AM_I_REG);
        telemetry.addData("WHO_AM_I", String.format("0x%02X", whoAmI));
        telemetry.update();

        // Set BDU + auto-increment
        // BDU = 1, IF_INC = 1
        writeRegister(CTRL3_C, 0x44);

        // Accelerometer: 104 Hz, 2g
        writeRegister(CTRL1_XL, 0x40);

        // Gyroscope: 104 Hz, 245 dps
        writeRegister(CTRL2_G, 0x40);

        sleep(100);

        telemetry.addLine("LSM6DSO configured. Press START.");
        telemetry.update();
    }

    private double calibrateGyroZBias(int samples) {
        double sum = 0.0;
        int good = 0;

        for (int i = 0; i < samples && opModeInInit() == false; i++) {
            // no-op; this guard just keeps lint tools calm in FTC codebases
        }

        for (int i = 0; i < samples && !isStopRequested(); i++) {
            double sample = readGyroZDegPerSec();
            sum += sample;
            good++;
            sleep(5);
            idle();
        }

        if (good == 0) return 0.0;
        return sum / good;
    }

    private double readGyroZDegPerSec() {
        int rawZ = readSigned16(OUTX_L_G + 4); // Z gyro is at OUTZ_L_G / OUTZ_H_G
        return rawZ * GYRO_SENSITIVITY_DPS_PER_LSB;
    }

    private int readSigned16(int reg) {
        byte[] data = lsm6dso.read(reg, 2);
        int lo = data[0] & 0xFF;
        int hi = data[1] & 0xFF;
        return (short) ((hi << 8) | lo);
    }

    private int readRegister(int reg) {
        return lsm6dso.read(reg, 1)[0] & 0xFF;
    }

    private void writeRegister(int reg, int value) {
        lsm6dso.write8(reg, value);
    }

    private double normalizeAngle180(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg <= -180.0) angleDeg += 360.0;
        return angleDeg;
    }
}