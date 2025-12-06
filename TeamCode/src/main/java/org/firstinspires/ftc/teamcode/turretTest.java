package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Turret.turretMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="turretTest", group="Linear Opmode")
public class turretTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Boolean rightBumperPressed = false;

    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        Turret turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("mode: ", turret.mode);
            telemetry.update();

            if (turret.mode == turretMode.AUTO) {
                if (gamepad1.right_bumper && !rightBumperPressed) {
                    rightBumperPressed = true;
                    turret.manualMode(Turret.turretDirection.STOP);
                } else {
                    turret.autoMode();
               }
            } else if (turret.mode == turretMode.MANUAL) {
                if (gamepad1.right_bumper && !rightBumperPressed) {
                    rightBumperPressed = true;
                    turret.autoMode();
                } else if (gamepad1.dpad_left) {
                   turret.manualMode(Turret.turretDirection.LEFT);
               } else if (gamepad1.dpad_right) {
                   turret.manualMode(Turret.turretDirection.RIGHT);
               } else {
                   turret.stop();
               }
           }
            if (!gamepad1.right_bumper && rightBumperPressed == true) {
                rightBumperPressed = false;
            }
        }
    }
}
