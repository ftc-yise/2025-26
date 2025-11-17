package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private ColorSensor color = null;

    private DcMotor intake = null;

    // timers and logging
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private PrintWriter logWriter = null;
    private String logFilePath = null;
    private double logInterval = 0.05; // 20Hz


    @Override
    public void runOpMode() throws InterruptedException {
        //class definition
        DriveClass drive = new DriveClass(hardwareMap);

        // hardware map
        color = hardwareMap.get(ColorSensor.class, "color");

        intake = hardwareMap.get(DcMotor.class, "intake");

        runtime.reset();
        logTimer.reset();

        telemetry.addLine("BallBotFieldDrive - ready (D-Pad override enabled)");
        telemetry.addLine("Left stick = field translation | Right X = rotation | D-Pad = perfect vectors");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //drive class
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1);

            if (gamepad1.right_trigger > 0.75) {
                intake.setPower(.6);
            } else if (gamepad1.left_trigger > .75) {
                intake.setPower(-.6);
            } else {
                intake.setPower(0);
            }

            // START+BACK -> start logging
            if (gamepad1.start && gamepad1.back && logWriter == null) {
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

            // telemetry
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

