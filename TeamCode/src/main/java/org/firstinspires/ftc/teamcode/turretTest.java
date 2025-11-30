package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.yise.Turret;
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

            if (gamepad1.b) {
                turret.manualTurn(Turret.turretDirection.LEFT);
            } else if (gamepad1.x){
                turret.manualTurn(Turret.turretDirection.RIGHT);
            } else {
                turret.stop();
            }

        }
    }
}
