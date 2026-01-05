package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class Turret {
    double mySlope = 0.15;
    double myOffset = 0.25;
    double tx = 0.0;
    public int homePos = 295;       // Right side home
    public int farLimit = -1075;    // Left side home
    public int centerPos = (homePos + farLimit) / 2; // Approx. calculated center

    // --- PID CONTROL CONSTANTS  ---
    public static double kP = 0.05;
    public static double kD = 0.01;
    public static double kI = 0.0;
    public static double AUTO_MAX_POWER = 0.7;
    public static double AUTO_MIN_POWER_FLOOR = 0.15;
    public static double TARGET_TOLERANCE_DEG = 1.0;
    public static double FINAL_DIRECTION_MULTIPLIER = -1.0;

    // --- ANALOG MANUAL CONTROL CONSTANTS  ---
    public static double MAX_MANUAL_POWER = 1.0;
    public static double ANALOG_POWER_CURVE_EXPONENT = 2.0;
    public static double MIN_ANALOG_POWER_FLOOR = 0.1;

    // --- PID STATE VARIABLES ---
    private double lastError = 0.0;
    private long lastTime = System.currentTimeMillis();
    private double integralSum = 0.0;

    // --- CLASS VARIABLES ---
    LLResult result = null;
    public double turretPower = 0.0;
    public double myTx = 0.0;
    public DcMotor turret;
    public Limelight3A limelight;
    public DigitalChannel limit; // Digital device for limit switch instead of push sensor because I get more advanced control
    public Telemetry telemetry;

    public enum turretAlliance {RED, BLUE}
    public turretAlliance currentAlliance;

    public enum turretDirection {LEFT, RIGHT, STOP}

    public enum turretMode {AUTO, MANUAL}

    public turretMode mode;

    public Turret(HardwareMap hardwareMap, turretAlliance alliance, Telemetry telem) {
        telemetry = telem;
        currentAlliance = alliance;

        turret = hardwareMap.get(DcMotor.class, "turret");

        // RESET ENCODER but stay in WITHOUT_ENCODER mode
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);

        // Limit Switch Setup
        limit = hardwareMap.get(DigitalChannel.class, "limit");
        limit.setMode(DigitalChannel.Mode.INPUT);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        if (currentAlliance == turretAlliance.RED) {
            limelight.pipelineSwitch(4);
        } else if (currentAlliance == turretAlliance.BLUE) {
            limelight.pipelineSwitch(3);
        }
        mode = turretMode.MANUAL;
        lastTime = System.currentTimeMillis();
    }

    // Limit switch safety
    private double applySafety(double power) {
        // Digital sensors are TRUE when open, FALSE when pressed
        boolean isPressed = !limit.getState();

        if (isPressed) {
            int pos = turret.getCurrentPosition();
            if (pos > 10 && power > 0) return 0;
            if (pos < -10 && power < 0) return 0;
        }
        return Range.clip(power, -1.0, 1.0);
    }

    public void changeMode() {
        if (mode == turretMode.AUTO) {
            mode = turretMode.MANUAL;
            stop();
            resetPD();
        } else if (mode == turretMode.MANUAL) {
            mode = turretMode.AUTO;
        }
    }

    public void manualMode(turretDirection direction) {
        mode = turretMode.MANUAL;
        double p = 0;
        switch (direction) {
            case LEFT:
                p = -0.4;
                break;
            case RIGHT:
                p = 0.4;
                break;
            case STOP:
                p = 0;
                break;
        }
        turret.setPower(applySafety(p));
    }

    public void manualControl(double power) {
        mode = turretMode.MANUAL;

        // --- POWER CURVE MATH STUFF ---
        double curbedPower = Math.signum(power) * Math.pow(Math.abs(power), ANALOG_POWER_CURVE_EXPONENT);
        double finalPower = curbedPower * MAX_MANUAL_POWER;

        if (Math.abs(finalPower) > 0.0 && Math.abs(finalPower) < MIN_ANALOG_POWER_FLOOR) {
            finalPower = Math.signum(finalPower) * MIN_ANALOG_POWER_FLOOR;
        }

        turretPower = applySafety(finalPower);
        turret.setPower(turretPower);
    }

    public void autoMode() {
        mode = turretMode.AUTO;

        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            myTx = result.getTx();
            turretPower = getTurretPower(myTx, myOffset, mySlope);
            //telemetry.addData("Ty=", myTy);
        } else {
            turretPower = 0;
        }
        //telemetry.addData("Power=", turretPower);
        //telemetry.update();
        turret.setPower(-turretPower);
    }

    public void autoModePid() {
        mode = turretMode.AUTO;
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double rawError_tx = result.getTx();
            double currentError = rawError_tx * -1.0;
            myTx = currentError;

            // --- PID MATH ---
            double pdPower = calculatePDPower(currentError) * FINAL_DIRECTION_MULTIPLIER;
            turretPower = applySafety(pdPower);

        } else {
            turretPower = 0;
            resetPD();
        }
        turret.setPower(turretPower);
    }

    public void stop() {
        turret.setPower(0);
    }

    private double getTurretPower(double tx, double myOffset, double mySlope) {
        double myPower = 0.0;

        if (tx < 0) {
            myPower = -.2 * (tx * mySlope + myOffset);
            if (myPower > 0.7) {
                myPower = AUTO_MAX_POWER;
            } else if (myPower < 0.35) {
                myPower = AUTO_MIN_POWER_FLOOR;
            }
            return myPower;
        } else if (tx > 0) {
            myPower = -.2 * (tx * mySlope - myOffset);
            if (myPower < -0.7) {
                myPower = -1 * AUTO_MAX_POWER;
            } else if (myPower > -0.35) {
                myPower = -1 * AUTO_MIN_POWER_FLOOR;
            }
            return myPower;
        } else {
            return myPower;
        }
    }

    public void setHome() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void resetPD() {
        lastError = 0.0;
        lastTime = System.currentTimeMillis();
    }

    private double calculatePDPower(double currentError) {
        if (Math.abs(currentError) < TARGET_TOLERANCE_DEG) {
            resetPD();
            return 0.0;
        }

        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;

        double proportional = kP * currentError;
        double derivative = 0.0;
        if (deltaTime != 0) {
            derivative = kD * ((currentError - lastError) / deltaTime);
        }

        double outputPower = proportional + derivative;

        if (outputPower > 0 && Math.abs(outputPower) < AUTO_MIN_POWER_FLOOR) {
            outputPower = AUTO_MIN_POWER_FLOOR;
        } else if (outputPower < 0 && Math.abs(outputPower) < AUTO_MIN_POWER_FLOOR) {
            outputPower = -AUTO_MIN_POWER_FLOOR;
        }

        if (outputPower > AUTO_MAX_POWER) outputPower = AUTO_MAX_POWER;
        else if (outputPower < -AUTO_MAX_POWER) outputPower = -AUTO_MAX_POWER;

        lastError = currentError;
        lastTime = currentTime;

        return outputPower;
    }

    public double getTx() {
        myTx = tx;
        return myTx;
    }

    public int getID() {

        // Always refresh the result
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1; // No Limelight data
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return -1; // No AprilTags detected
        }

        // Return the first detected tag's ID
        {
            return fiducials.get(0).getFiducialId();
        }
    }

    public double getDistance() {
        double distance = 0.0;
        double a = 0.0;
        double b = 0.0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            //39.37 is a conversion from meters(what botpose gives) to inches
            double x = botpose.getPosition().x * 39.37;
            double y = botpose.getPosition().y * 39.37;
            telemetry.addData("x", x);
            telemetry.addData("y", y);

            if (currentAlliance == turretAlliance.RED) {
                telemetry.addData("Alliance", "RED");

                a = Math.abs(55 - y);
                b = Math.abs(-58 - x);
            } else if (currentAlliance == turretAlliance.BLUE) {
                telemetry.addData("Alliance", "BLUE");
                a = Math.abs(55 - y);
                b = Math.abs(-58 - x);
            }
            telemetry.addData("a", a);
            telemetry.addData("b", b);

            double c_sqrd = Math.pow(a, 2) + Math.pow(b, 2);
            telemetry.addData("c_sqrd", c_sqrd);
            distance = Math.sqrt(c_sqrd);
            telemetry.addData("distance", distance);

        }
        telemetry.update();
        return distance;
    }
}
