package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * BallBotMainDrive - Field-oriented baseline with D-Pad perfect vectors
 *
 * Fixed version that compiles and uses myOtos for heading/pose.
 */
@TeleOp(name = "BallBot", group = "Ball Bot")
public class BallBotMainDrive extends LinearOpMode {

    // hardware

    private DcMotor intake = null;
    private DcMotor turret = null;

    private CRServo hood = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

    private Servo footL = null;
    private Servo footR = null;


    // timers and logging
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private PrintWriter logWriter = null;
    private String logFilePath = null;
    private double logInterval = 0.05; // 20Hz

    boolean rightBumperPressed;
    boolean firstRun = true;

    @Override
    public void runOpMode() throws InterruptedException {
        //class definition
        DriveClass drive = new DriveClass(hardwareMap);
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap);
        org.firstinspires.ftc.teamcode.yise.Turret turret = new org.firstinspires.ftc.teamcode.yise.Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);


        // hardware map
        hood = hardwareMap.get(CRServo.class, "hood");

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        footL = hardwareMap.get(Servo.class, "footL");
        footR = hardwareMap.get(Servo.class, "footR");

        wallright.setDirection(CRServo.Direction.REVERSE);


        intake = hardwareMap.get(DcMotor.class, "intake");

        runtime.reset();
        logTimer.reset();

        telemetry.addLine("BallBotFieldDrive - ready (D-Pad override enabled)");
        telemetry.addLine("Left stick = field translation | Right X = rotation | D-Pad = perfect vectors");
        telemetry.update();
        hood.setPower(0);
        //shooter.update(false, false, true); // Y = FULL
        spin.goToSilo1();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(firstRun){
                shooter.setPower(.35);
                firstRun = false;
            }

            //spindexer
            if (gamepad2.a && !autoShoot.isBusy()) {
                shooter.update(false, false, true); // Y = FULL
                hood.setPower(-1);
                autoShoot.startCycle();
            } else if (gamepad2.x && !autoShoot.isBusy()){
                shooter.update(false, true, false); // X = FULL
                hood.setPower(1);
                autoShoot.startCycle();
            }
            autoShoot.update();

            //foot
            if (gamepad1.dpad_down){
                footL.setPosition(-1);
                footR.setPosition(1);
            } else if (gamepad1.dpad_up){
                footL.setPosition(1);
                footR.setPosition(-1);
            }

            //hood
            if (gamepad2.dpad_up && !autoShoot.isBusy()){
                hood.setPower(1);
            } else if (gamepad2.dpad_down && !autoShoot.isBusy()){
                hood.setPower(-1);
            } else if (!autoShoot.isBusy()) {
                hood.setPower(0);
            }

            //drive class
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1);

            //intake
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
            } else {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);

                if (!autoShoot.isBusy()) {
                    spin.setNeutral();
                }
            }



            //turret
            if (turret.mode == Turret.turretMode.AUTO) {
                if (gamepad2.right_bumper && !rightBumperPressed) {
                    rightBumperPressed = true;
                    turret.manualMode(Turret.turretDirection.STOP);
                } else {
                    turret.autoMode();
                }
            } else if (turret.mode == Turret.turretMode.MANUAL) {
                if (gamepad2.right_bumper && !rightBumperPressed) {
                    rightBumperPressed = true;
                    turret.autoMode();
                } else if (gamepad2.dpad_left) {
                    turret.manualMode(Turret.turretDirection.LEFT);
                } else if (gamepad2.dpad_right) {
                    turret.manualMode(Turret.turretDirection.RIGHT);
                } else {
                    turret.stop();
                }
            }
            if (!gamepad2.right_bumper && rightBumperPressed == true) {
                rightBumperPressed = false;
            }

            // START+BACK -> start logging
            if (gamepad1.start && gamepad1.back && logWriter != null) {
                try {
                    File dir = new File("/sdcard/FIRST");
                    if (!dir.exists()) dir.mkdirs();
                    logFilePath = "/sdcard/FIRST/field_only_log_" + System.currentTimeMillis() + ".csv";
                    logWriter = new PrintWriter(new FileWriter(new File(logFilePath), true));
                    logWriter.println("time_s,input_x,input_y,input_turn,trans_x,trans_y,rotation_cmd,lf,rf,lb,rb,pose_x,pose_y,pose_h,total_power");
                    logWriter.flush();
                    telemetry.addData("Log", "started: " + logFilePath);
                    telemetry.update();
                } catch (IOException e) {
                    telemetry.addData("Log Error", e.getMessage());
                    telemetry.update();
                }
            }

            //telemetry getter
            DriveClass.DriveTelemetry d = drive.getDriveTelemetry();

            // logging
            if (logWriter != null && logTimer.seconds() >= logInterval) {
                double now = runtime.seconds();
                double totalPower = Math.abs(d.lf) + Math.abs(d.rf) + Math.abs(d.lb) + Math.abs(d.rb);
                logWriter.printf("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
                        now, d.rawX, d.rawY, d.rawTurn, d.tx_field, d.ty_field, d.rotationCmd,
                        d.lf, d.rf, d.lb, d.rb, d.pose.x, d.pose.y, d.pose.h, totalPower);
                logWriter.flush();
                logTimer.reset();
            }

            ShooterClass.ShooterTelemetry shoot = shooter.getTelemetry();
            spin.update();
            Spindexer.TelemetryPacket spina = spin.getTelemetry();

            // telemetry
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", shoot.mode);
            telemetry.addData("Power", "%.2f", shooter.getPower());
            telemetry.addData("Velocity", "%.1f", shoot.velocity);

            telemetry.addLine("=== FIELD DRIVE ===");
            telemetry.addData("Speed Mode", d.currentSpeed);
            telemetry.addData("Heading (deg)", "%.2f", d.headingDeg);
            telemetry.addData("Inputs (raw)", "x:%.2f y:%.2f t:%.2f", d.rawX, d.rawY, d.rawTurn);
            telemetry.addData("FieldCmd (tx,ty)", "%.3f, %.3f", d.tx_field, d.ty_field);
            telemetry.addData("RobotCmd (rx,ry)", "%.3f, %.3f", d.robotX, d.robotY);
            telemetry.addData("Motor LF/RF/LB/RB", "%.3f / %.3f / %.3f / %.3f", d.lf, d.rf, d.lb, d.rb);
            telemetry.addData("Pose (x,y,h)", "%.2f, %.2f, %.2f", d.pose.x, d.pose.y, d.pose.h);
            telemetry.addData("Logging", logWriter != null ? ("ON: " + logFilePath) : "OFF");


            telemetry.addLine("=== SPINDEXER ===");
            telemetry.addData("Mode", spina.mode);
            telemetry.addData("Voltage", "%.3f", spina.voltage);
            telemetry.addData("Angle", "%.1f°", spina.currentAngle);
            telemetry.addData("Target", "%.1f°", spina.targetAngle);
            telemetry.addData("Error", "%.1f°", spina.angleError);
            telemetry.addData("Power", "%.2f", spina.appliedPower);

            telemetry.addLine("=== Turret ===");
            telemetry.addData("mode: ", turret.mode);
            telemetry.addData("power: ",turret.turret.getPower());
            telemetry.addData("ty: ", turret.myTy);
            telemetry.update();
        } // end while opModeIsActive

        // cleanup
        drive.stopAllMotors();
        if (logWriter != null) {
            logWriter.flush();
            logWriter.close();
            telemetry.addData("Log Saved", logFilePath);
            telemetry.update();
        }
    }
}

