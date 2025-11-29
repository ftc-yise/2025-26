package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;

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

    private Servo lift = null;
    private CRServo hood = null;
    private CRServo walleft = null;
    private CRServo wallright = null;


    // timers and logging
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private PrintWriter logWriter = null;
    private String logFilePath = null;
    private double logInterval = 0.05; // 20Hz

    boolean RightBumperPressed;
    boolean LeftBumperPressed;

    @Override
    public void runOpMode() throws InterruptedException {
        //class definition
        DriveClass drive = new DriveClass(hardwareMap);
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);


        // hardware map
        hood = hardwareMap.get(CRServo.class, "hood");
        lift = hardwareMap.get(Servo.class, "lift");

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");

        runtime.reset();
        logTimer.reset();

        telemetry.addLine("BallBotFieldDrive - ready (D-Pad override enabled)");
        telemetry.addLine("Left stick = field translation | Right X = rotation | D-Pad = perfect vectors");
        telemetry.update();
        hood.setPower(0);
        lift.setPosition(0);
        spin.startSequence();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //spindexer
            if (gamepad1.right_bumper && !RightBumperPressed) {
                RightBumperPressed = true;

                if (spin.mode == Spindexer.Mode.SILO_1) {
                    spin.goToSilo2();
                } else if (spin.mode == Spindexer.Mode.SILO_2) {
                    spin.goToSilo3();
                } else {
                    spin.goToSilo1();
                }

            } else if (!gamepad1.right_bumper && RightBumperPressed) {
                RightBumperPressed = false;

            }

            //hood
            if (gamepad1.right_stick_button){
                hood.setPower(1);
            } else if (gamepad1.left_stick_button){
                hood.setPower(-1);
            } else {
                hood.setPower(0);
            }

            //lift
            if (gamepad1.left_bumper && !LeftBumperPressed) {
                LeftBumperPressed = true;

                lift.setPosition(lift.getPosition() == 0.75 ? 0 : 0.75);

            } else if (!gamepad1.left_bumper && LeftBumperPressed) {
                LeftBumperPressed = false;
            }



            //drive class
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1);

            //intake
            if (gamepad1.right_trigger > 0.75) {
                intake.setPower(.6);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(.35);
            } else if (gamepad1.left_trigger > .75) {
                intake.setPower(-.6);
                spin.setManual(-.35);
            } else {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
            }

            //Shooter caller

            shooter.update(
                    gamepad1.a,   // STOP
                    gamepad1.b,   // IDLE
                    gamepad1.x,   // LOW
                    gamepad1.y    // FULL
            );

            //turret
            if (gamepad1.back){
                turret.setPower(.5);
            } else if (gamepad1.start){
                turret.setPower(-.5);
            } else {
                turret.setPower(0);
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

            ShooterClass.ShooterTelemetry s = shooter.getTelemetry();

            // telemetry
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", s.mode);
            telemetry.addData("Power", "%.2f", s.appliedPower);
            telemetry.addData("Velocity", "%.1f", s.velocity);

            telemetry.addLine("=== FIELD DRIVE ===");
            telemetry.addData("Speed Mode", d.currentSpeed);
            telemetry.addData("Heading (deg)", "%.2f", d.headingDeg);
            telemetry.addData("Inputs (raw)", "x:%.2f y:%.2f t:%.2f", d.rawX, d.rawY, d.rawTurn);
            telemetry.addData("FieldCmd (tx,ty)", "%.3f, %.3f", d.tx_field, d.ty_field);
            telemetry.addData("RobotCmd (rx,ry)", "%.3f, %.3f", d.robotX, d.robotY);
            telemetry.addData("Motor LF/RF/LB/RB", "%.3f / %.3f / %.3f / %.3f", d.lf, d.rf, d.lb, d.rb);
            telemetry.addData("Pose (x,y,h)", "%.2f, %.2f, %.2f", d.pose.x, d.pose.y, d.pose.h);
            telemetry.addData("Logging", logWriter != null ? ("ON: " + logFilePath) : "OFF");
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

