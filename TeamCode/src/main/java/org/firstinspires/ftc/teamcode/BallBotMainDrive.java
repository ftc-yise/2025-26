package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Parameters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    // ------------------------------

    private DcMotor intake = null;
    private CRServo hood = null;
    private CRServo walleft = null;
    private CRServo wallright = null;
    private Servo footL = null;
    private Servo footR = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private PrintWriter logWriter = null;
    private String logFilePath = null;
    private double logInterval = 0.05;

    boolean firstRun = true;
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveClass drive = new DriveClass(hardwareMap);
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
        }
        Turret turret = new Turret(hardwareMap, alliance, telemetry);

        hood = hardwareMap.get(CRServo.class, "hood");
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        footL = hardwareMap.get(Servo.class, "footL");
        footR = hardwareMap.get(Servo.class, "footR");
        wallright.setDirection(CRServo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");

        hood.setPower(0);
        spin.goToSilo1();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(firstRun){
                shooter.setPower(.5);
                firstRun = false;
            }

            // --- DRIVE & SPEED TOGGLE ---
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1);

            // --- SHOOTING & SPINDEXOR ---
            if (gamepad2.a && !autoShoot.isBusy()) {
                shooter.update(false, false, true);
                hood.setPower(-.7);
                autoShoot.startCycle();
            } else if (gamepad2.x && !autoShoot.isBusy()){
                shooter.update(false, true, false);
                hood.setPower(1);
                autoShoot.startCycle();
            }
            autoShoot.update();

            // --- INTAKE & WALL WHEELS ---
            if (gamepad1.right_trigger > 0.75) {
                intake.setPower(.6);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(.1);
            } else if (gamepad1.left_trigger > .75) {
                intake.setPower(-.6);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(.2);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-.6);
                walleft.setPower(-1);
                wallright.setPower(-1);
                spin.setManual(0);
            } else {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
                if (!autoShoot.isBusy()) spin.setNeutral();
            }

            // --- HOOD & FOOT ---
            if (gamepad1.dpad_down) { footL.setPosition(-1); footR.setPosition(1); }
            else if (gamepad1.dpad_up) { footL.setPosition(1); footR.setPosition(-1); }

            if (gamepad2.dpad_up && !autoShoot.isBusy()) hood.setPower(1);
            else if (gamepad2.dpad_down && !autoShoot.isBusy()) hood.setPower(-.6);
            else if (!autoShoot.isBusy()) hood.setPower(0);


            // =========================================================
            // --- TURRET SYSTEM (INTEGRATED STATE MACHINE) ---
            // =========================================================

            // 1. Mode Toggle (Options) & Homing (Share)
            if (gamepad2.options && !modeTogglePressed) {
                turret.changeMode();
                currentSnapState = SnapState.INACTIVE;
                modeTogglePressed = true;
            }
            if (!gamepad2.options) modeTogglePressed = false;

            if (gamepad2.share) {
                currentSnapState = SnapState.HOMING_ROUTINE;
                homingTimer.reset();
            }

            // 2. Input Detection (Manual Mode only)
            if (turret.mode == Turret.turretMode.MANUAL) {
                double turretManualTrigger = gamepad2.right_trigger - gamepad2.left_trigger;

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
            else if (currentSnapState == SnapState.INACTIVE && Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) <= 0.05) {
                turret.stop();
            }


            // --- LOGGING & TELEMETRY UPDATES ---
            spin.update();
            DriveClass.DriveTelemetry d = drive.getDriveTelemetry();
            ShooterClass.ShooterTelemetry shoot = shooter.getTelemetry();
            Spindexer.TelemetryPacket spina = spin.getTelemetry();

            telemetry.addLine("=== TURRET: ===");
            telemetry.addData("ENC:", turret.turret.getCurrentPosition());
            telemetry.addData("STATE:", currentSnapState);
            telemetry.addData("MODE:", turret.mode);

            telemetry.addLine("=== SHOOTER: ===");
            telemetry.addData("POWER:", "%.2f", shooter.getPower());
            telemetry.addData("VELOCITY:", "%.1f", shoot.velocity);

            telemetry.addLine("=== CHASSIS ===");
            telemetry.addData("HEADING (deg):", "%.2f", d.headingDeg);
            telemetry.addData("POSE (x,y,h):", "%.2f, %.2f, %.2f", d.pose.x, d.pose.y, d.pose.h);

            telemetry.update();

        } // end while

        drive.stopAllMotors();
        if (logWriter != null) { logWriter.flush(); logWriter.close(); }
    }
}