package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.Turret.turretMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="turretTest", group="Linear Opmode")
public class turretTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        Turret turret = new Turret(hardwareMap);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

           if (gamepad1.right_bumper && (turret.mode == turretMode.AUTO)) {
               turret.changeMode();
           } else if (gamepad1.right_bumper && (turret.mode == turretMode.MANUAL)) {
               turret.autoMode();
           }
           if (turret.mode == turretMode.MANUAL) {
               if (gamepad1.dpad_left) {
                   turret.manualMode(Turret.turretDirection.LEFT);
               } else if (gamepad1.dpad_right) {
                   turret.manualMode(Turret.turretDirection.RIGHT);
               } else {
                   turret.stop();
               }
           }
        }
    }
}
