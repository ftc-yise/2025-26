package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;

@Autonomous(name="Auto Far Side Shoot", group="auto")
public class FarShootAuto extends LinearOpMode {
    private CRServo hood = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        DriveClass drive = new DriveClass(hardwareMap);
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap);
        org.firstinspires.ftc.teamcode.yise.Turret turret = new org.firstinspires.ftc.teamcode.yise.Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        hood = hardwareMap.get(CRServo.class, "hood");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        waitForStart();
        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 15)) {
            shooter.update(false, false, true); // Y = FULL
            hood.setPower(-1);

            if (runtime.seconds() > 3) {
                if (!autoShoot.isBusy()) {
                    autoShoot.startCycle();
                }
                autoShoot.update();
                spin.update();
                turret.autoMode();
            }
        }

        // Step 2:  Spin right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .8)) {
            drive.setAutoPower(-1,1,1,-1);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 4:  Stop
        drive.setAutoPower(0,0,0,0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
