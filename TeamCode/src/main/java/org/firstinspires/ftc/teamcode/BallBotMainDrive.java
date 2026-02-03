package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.lifter;
import org.firstinspires.ftc.teamcode.yise.Ledclass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

@TeleOp(name = "BallBot", group = "Ball Bot")
public class BallBotMainDrive extends LinearOpMode {

    // --- Turret Logic Variables ---
    private enum SnapState { INACTIVE, SNAPPING_LEFT, PUSHING_LEFT, SNAPPING_RIGHT, PUSHING_RIGHT, GOING_HOME, HOMING_ROUTINE }
    private SnapState currentSnapState = SnapState.INACTIVE;
    private ElapsedTime homingTimer = new ElapsedTime();
    private ElapsedTime snapTimer = new ElapsedTime();
    private boolean modeTogglePressed = false;

    private static final int LEFT_LIMIT = -1370;
    private static final int CENTER_TARGET = -685;
    private static final int TOLERANCE = 10;
    private boolean shooting = false;

    // ------------------------------

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;
    private Servo footL = null;
    private Servo footR = null;

    private ColorSensor middle = null;
    private ColorSensor backLeft = null;
    private ColorSensor backRight = null;

    public Ledclass led1;
    public Ledclass led2;
    public Ledclass led3;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private PrintWriter logWriter = null;
    private String logFilePath = null;
    private double logInterval = 0.05;
    public double time = runtime.seconds();

    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }
    private ShotPatternManager.ShotPattern activePattern = null;
    boolean firstRun = true;
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveClass drive = new DriveClass(hardwareMap);
        ShotPatternManager patternMgr = new ShotPatternManager();
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        autoShoot.setPatternManager(patternMgr);
        Parameters parem = new Parameters();
        parem.autonomous = Parameters.AUTONOMOUS.NO;

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
        }

        Turret turret = new Turret(hardwareMap, alliance, telemetry);

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        footL = hardwareMap.get(Servo.class, "footL");
        footR = hardwareMap.get(Servo.class, "footR");
        wallright.setDirection(CRServo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");

        middle = hardwareMap.get(ColorSensor.class, "middlecolorsensor");
        backLeft = hardwareMap.get(ColorSensor.class, "BLcolorsensor");
        backRight = hardwareMap.get(ColorSensor.class, "BRcolorsensor");

        led1 = new Ledclass(hardwareMap, "led1");
        led2 = new Ledclass(hardwareMap, "led2");
        led3 = new Ledclass(hardwareMap, "led3");

        hood.stop();

        spin.initSilos();

        spin.goToSilo2();
        lifter.setDown();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            time = runtime.seconds();

            if (Parameters.allianceColor == Parameters.Color.RED) {
                alliance = Turret.turretAlliance.RED;
            } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
                alliance = Turret.turretAlliance.BLUE;
            }

            // --- DRIVE & SPEED TOGGLE ---
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1, false);

            // --- SHOOTING & SPINDEXOR ---
            if (gamepad2.a && !autoShoot.isBusy()) {
                shooter.update(false, false, true);
                hood.setTarget(60);

                if (activePattern != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(activePattern.sequence);
                }

                autoShoot.startCycle();
                shooting = true;
                //spin.goToSilo1();
            }
            else if (gamepad2.x && !autoShoot.isBusy()){
                shooter.update(false, true, false);
                hood.setTarget(15); // e.g. 0
                if (activePattern != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(activePattern.sequence);
                }

                autoShoot.startCycle();
                shooting = true;
                //spin.goToSilo2();
            }
            autoShoot.update();
            // --- SHOOTING & SPINDEXOR (forced override when holding A or X) ---
            /*if (gamepad2.a) {
                // start forced-fire if not already
                shooter.update(false, false, true);    // shooter high goal
                hood.setTarget(60);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            } else if (gamepad2.x) {
                shooter.update(false, true, false);    // shooter lower goal
                hood.setTarget(15);
                if (!autoShoot.forceShooting && !autoShoot.isBusy()) {
                    autoShoot.startForcedCycle();
                }
            } else {
                // if neither held, and we're in forced-mode, stop forced mode
                if (autoShoot.forceShooting) {
                    autoShoot.stopForcedCycle();
                }
                // normal idle behavior handled elsewhere
            }
            autoShoot.update();*/

            //vision/pattern things

            if (gamepad1.y) {
                turret.limelight.pipelineSwitch(2);

                int tagId = turret.getID();
                ShotPatternManager.ShotPattern p = patternFromTag(tagId);
                if (p != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(p.sequence);
                }

            }
            else {
                if (Parameters.allianceColor == Parameters.Color.RED) {
                    alliance = Turret.turretAlliance.RED;
                    turret.limelight.pipelineSwitch(4);
                } else {
                    alliance = Turret.turretAlliance.BLUE;
                    turret.limelight.pipelineSwitch(3);
                }
            }


            if ((turret.getID() == 20 || turret.getID() == 24)){
                led1.setBlue();
                led2.setBlue();
                led3.setBlue();
            } else if (time > 1) {
                led1.setOff();
                led2.setOff();
                led3.setOff();
                runtime.reset();
            }

            // --- INTAKE & WALL WHEELS ---
            if (gamepad1.right_trigger > 0.75 && !shooting) {
                intake.setPower(1);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(.1);
            } else if (gamepad1.left_trigger > .75 && !shooting) {
                intake.setPower(-.6);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(.2);
            } else if (gamepad1.right_bumper && !shooting) {
                intake.setPower(-.6);
                walleft.setPower(-1);
                wallright.setPower(-1);
                spin.setManual(0);
            } else {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
                if (!autoShoot.isBusy()) {
                    spin.setNeutral();
                        shooting = false;
                }
            }

            // --- HOOD & FOOT ---
            if (gamepad1.dpad_down) {
                footL.setPosition(-1);
                footR.setPosition(1);
            }
            else if (gamepad1.dpad_up) {
                footL.setPosition(1);
                footR.setPosition(-1);
            }

            if (gamepad2.dpad_up && !autoShoot.isBusy()) {
                hood.setTarget(24); // e.g. 42.0
                lifter.setUp();
                shooting = true;
            }
            else if (gamepad2.dpad_down && !autoShoot.isBusy()) {
                hood.setTarget(0); // e.g. 42.0
                lifter.setDown();
                shooting = true;
            }


            if (gamepad2.right_bumper && !autoShoot.isBusy()) {
                spin.setManual(0.09);
                shooting = true;
            }
            else if (gamepad2.left_bumper && !autoShoot.isBusy()) {
                spin.setManual(-0.09);
                shooting = true;
            }


            // =========================================================
            // --- TURRET SYSTEM (INTEGRATED STATE MACHINE) ---
            // =========================================================

            // 1. Mode Toggle (y) & Homing (Share)
            if (gamepad2.y && !modeTogglePressed) {
                turret.changeMode();
                currentSnapState = SnapState.INACTIVE;
                modeTogglePressed = true;
            }
            if (!gamepad2.y) {
                modeTogglePressed = false;
            }

            if (gamepad2.share) {
                currentSnapState = SnapState.HOMING_ROUTINE;
                homingTimer.reset();
            }

            // 2. Input Detection (Manual Mode only)
            if (turret.mode == Turret.turretMode.MANUAL) {
                double turretManualTrigger = (gamepad2.left_trigger - gamepad2.right_trigger) * -.8;

                if (Math.abs(turretManualTrigger) > 0.05) {
                    currentSnapState = SnapState.INACTIVE;
                    turret.manualControl(turretManualTrigger);
                }
                else if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    currentSnapState = SnapState.GOING_HOME;
                }
                else if (gamepad2.right_bumper && currentSnapState != SnapState.SNAPPING_RIGHT && currentSnapState != SnapState.PUSHING_RIGHT) {
                    currentSnapState = SnapState.SNAPPING_RIGHT;
                }
                else if (gamepad2.left_bumper && currentSnapState != SnapState.SNAPPING_LEFT && currentSnapState != SnapState.PUSHING_LEFT) {
                    currentSnapState = SnapState.SNAPPING_LEFT;
                }
            }

            // 3. State Machine Execution
            if (turret.mode == Turret.turretMode.AUTO) {
                turret.autoMode();
                turret.mode = Turret.turretMode.AUTO;
            }
            else if (currentSnapState == SnapState.HOMING_ROUTINE) {
                if (gamepad2.share) {
                    if (!turret.limit.getState()) {
                        turret.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        turret.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turret.manualControl(0.4);
                    } else {
                        double curvePower = 0.9 - (homingTimer.seconds() * 0.33);
                        turret.manualControl(Range.clip(curvePower, 0.4, 0.9));
                    }
                } else {
                    currentSnapState = SnapState.INACTIVE;
                    turret.stop();
                }
            }
            else if (currentSnapState == SnapState.GOING_HOME) {
                int currentPos = turret.turret.getCurrentPosition();
                int error = CENTER_TARGET - currentPos;
                if (Math.abs(error) > TOLERANCE) {
                    double homePower = (error > 0) ? 1.0 : -1.0;
                    if (Math.abs(error) < 200) homePower *= 0.4;
                    turret.manualControl(homePower);
                } else {
                    turret.stop();
                    currentSnapState = SnapState.INACTIVE;
                }
            }
            else if (currentSnapState == SnapState.SNAPPING_RIGHT) {
                turret.manualControl(1.0);
                if (!turret.limit.getState()) { snapTimer.reset(); currentSnapState = SnapState.PUSHING_RIGHT; }
            }
            else if (currentSnapState == SnapState.PUSHING_RIGHT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(0.5);
                else { turret.stop(); currentSnapState = SnapState.INACTIVE; }
            }
            else if (currentSnapState == SnapState.SNAPPING_LEFT) {
                int currentPos = turret.turret.getCurrentPosition();
                turret.manualControl(-1.0);
                if (currentPos <= LEFT_LIMIT) { snapTimer.reset(); currentSnapState = SnapState.PUSHING_LEFT; }
            }
            else if (currentSnapState == SnapState.PUSHING_LEFT) {
                if (snapTimer.seconds() < 1.0) turret.manualControl(-0.5);
                else { turret.stop(); currentSnapState = SnapState.INACTIVE; }
            }
            else if (turret.mode == Turret.turretMode.MANUAL && currentSnapState == SnapState.INACTIVE && Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) <= 0.05) {
                turret.stop();
            }


            if (logWriter == null) {
                try {
                    File dir = new File("/sdcard/FIRST");
                    if (!dir.exists()) dir.mkdirs();

                    logFilePath = "/sdcard/FIRST/telemetry_" + System.currentTimeMillis() + ".csv";
                    logWriter = new PrintWriter(new FileWriter(logFilePath));

                    logWriter.println(
                            "time_s," +
                                    "input_x,input_y,input_turn," +
                                    "trans_x,trans_y,rotation_cmd," +
                                    "lf,rf,lb,rb,total_power," +
                                    "pose_x,pose_y,pose_h," +
                                    "spx_mode,spx_currentAngle,spx_targetAngle,spx_error,spx_appliedPower," +
                                    "silo1,silo2,silo3"
                    );

                    logWriter.flush();
                    logTimer.reset();

                    telemetry.addData("LOG", "Started");
                    telemetry.addData("File", logFilePath);
                    telemetry.update();

                    sleep(300); // debounce
                } catch (IOException e) {
                    telemetry.addData("LOG ERROR", e.getMessage());
                    telemetry.update();
                }
            }


            if (gamepad1.a) {
                patternMgr.clear();
            }

            //telemetry getter
            DriveClass.DriveTelemetry d = drive.getDriveTelemetry();
            // --- Update spindexer & autoShoot ---
            hood.update();
            Hood.TelemetryPacket H = hood.getTelemetry();
            lifter.TelemetryPacket l = lifter.getTelemetry();

// --- TELEMETRY ---
// SHOOTER
            shooter.updateTelemetry();
            ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", s.mode);
            telemetry.addData("Target RPM", "%.2f", s.targetRPM);
            telemetry.addData("Current RPM", "%.1f", s.currentRPM);
            telemetry.addData("Error RPM", "%.1f", s.errorRPM);
            telemetry.addData("volt", s.motorPower);
            telemetry.addData("pose", s.pose);
            telemetry.addData("spin up time", s.spinupTimeSec);

// DRIVE
            telemetry.addLine("=== FIELD DRIVE ===");
            telemetry.addData("Speed Mode", d.currentSpeed);
            telemetry.addData("Heading (deg)", "%.2f", d.headingDeg);
            telemetry.addData("Inputs (raw)", "x:%.2f y:%.2f t:%.2f", d.rawX, d.rawY, d.rawTurn);
            telemetry.addData("FieldCmd (tx,ty)", "%.3f, %.3f", d.tx_field, d.ty_field);
            telemetry.addData("RobotCmd (rx,ry)", "%.3f, %.3f", d.robotX, d.robotY);
            telemetry.addData("Motor LF/RF/LB/RB", "%.3f / %.3f / %.3f / %.3f", d.lf, d.rf, d.lb, d.rb);
            telemetry.addData("Pose (x,y,h)", "%.2f, %.2f, %.2f", d.pose.x, d.pose.y, d.pose.h);

// SPINDEXER
            telemetry.addLine("=== SPINDEXER ===");
            spin.sampleSensorsNow();
            spin.update();             // 2️⃣ process logic
            Spindexer.TelemetryPacket spina = spin.getTelemetry(); // 3️⃣ snapshot
            telemetry.addData("Mode", spina.mode);
            telemetry.addData("Voltage", "%.3f", spina.voltage);
            telemetry.addData("Angle", "%.1f°", spina.currentAngle);
            telemetry.addData("Target", "%.1f°", spina.targetAngle);
            telemetry.addData("Error", "%.1f°", spina.angleError);
            telemetry.addData("Power", "%.2f", spina.appliedPower);

// TURRET
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.mode);
            telemetry.addData("Power", turret.turretPower);
            telemetry.addData("pose", turret.getPose());
            telemetry.addData("id", turret.getID());
            telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

// SILOS
            telemetry.addLine("=== SILOS ===");
            Spindexer.BallColor[] silos = spina.siloColors;
            for (int i = 0; i < silos.length; i++) {
                String label = "Silo " + (i+1);
                // Highlight the current silo
                telemetry.addData(label, silos[i]);
            }

// COLOR SENSORS
            telemetry.addLine("=== COLOR SENSORS ===");
            telemetry.addLine("Middle");
            telemetry.addData("Blue", middle.blue());
            telemetry.addData("Red", middle.red());
            telemetry.addData("Green", middle.green());

            telemetry.addLine("Back Left");
            telemetry.addData("Blue", backLeft.blue());
            telemetry.addData("Red", backLeft.red());
            telemetry.addData("Green", backLeft.green());

            telemetry.addLine("Back Right");
            telemetry.addData("Blue", backRight.blue());
            telemetry.addData("Red", backRight.red());
            telemetry.addData("Green", backRight.green());
//lift

            telemetry.addLine("=== LIFT ===");
            telemetry.addData("pose", l.position);
            telemetry.addData("volt", l.voltage);
            telemetry.addData("err", l.error);
            telemetry.addData("mode", l.mode);
//hood

            telemetry.addLine("=== HOOD ===");
            telemetry.addData("Mode", H.mode);
            telemetry.addData("Voltage", "%.3f", H.voltage);
            telemetry.addData("Angle", "%.1f°", H.currentAngle);
            telemetry.addData("Target", "%.1f°", H.targetAngle);
            telemetry.addData("Error", "%.1f°", H.angleError);
            telemetry.addData("Power", "%.2f", H.appliedPower);
            telemetry.update();

            if (logWriter != null && logTimer.seconds() >= 0.1) {

                double now = runtime.seconds();

                Spindexer.TelemetryPacket spx = spin.getTelemetry();

                double totalPower =
                        Math.abs(d.lf) + Math.abs(d.rf) +
                                Math.abs(d.lb) + Math.abs(d.rb);

                logWriter.printf(
                        "%.3f," +
                                "%.4f,%.4f,%.4f," +
                                "%.4f,%.4f,%.4f," +
                                "%.4f,%.4f,%.4f,%.4f,%.4f," +
                                "%.4f,%.4f,%.4f," +
                                "%s,%.2f,%.2f,%.2f,%.3f," +
                                "%s,%s,%s%n",

                        now,
                        d.rawX, d.rawY, d.rawTurn,
                        d.tx_field, d.ty_field, d.rotationCmd,
                        d.lf, d.rf, d.lb, d.rb, totalPower,
                        d.pose.x, d.pose.y, d.pose.h,
                        spx.mode,
                        spx.currentAngle,
                        spx.targetAngle,
                        spx.angleError,
                        spx.appliedPower,
                        spx.siloColors[0],
                        spx.siloColors[1],
                        spx.siloColors[2]
                );

                logWriter.flush();
                logTimer.reset();
            }


        } // end while opModeIsActive

        if (logWriter != null) {
            logWriter.flush();
            logWriter.close();
            telemetry.addData("LOG", "Saved");
            telemetry.addData("File", logFilePath);
            telemetry.update();
        }



    }

}