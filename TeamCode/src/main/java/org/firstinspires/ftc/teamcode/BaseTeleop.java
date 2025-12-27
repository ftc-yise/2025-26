package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;

@TeleOp(name="BaseTeleop", group="Linear Opmode")
public class BaseTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;

    @Override
    public void runOpMode() {
        DriveClass drive = new DriveClass(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // --- DRIVE & SPEED TOGGLE ---
            drive.handleSpeedToggle(gamepad1);
            drive.updateMotors(gamepad1, true);

            // put your teleop code here
            if (gamepad1.right_trigger > 0.75) {
                intake.setPower(.6);

            } else if (gamepad1.left_trigger > .75) {
                intake.setPower(-.6);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-.6);
            } else {
                intake.setPower(0);
            }
        }
        }
}
