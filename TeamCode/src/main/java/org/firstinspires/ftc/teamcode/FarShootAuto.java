package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name="Auto Far Side Shoot", group="auto")
public class FarShootAuto extends LinearOpMode {
    private CRServo hood = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static double     directioninalMulti    = 1;
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;

    @Override
    public void runOpMode() {

        DriveClass drive = new DriveClass(hardwareMap);
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        org.firstinspires.ftc.teamcode.yise.Turret turret = new org.firstinspires.ftc.teamcode.yise.Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        hood = hardwareMap.get(CRServo.class, "hood");


        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            directioninalMulti = 1;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            directioninalMulti = -1;
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        waitForStart();
        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        sleep((long) (Parameters.WAIT * 1000));
        // Step 1:  Drive forward for 3 seconds
        runtime.reset();

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
        }

        while (opModeIsActive() && (runtime.seconds() < 10)) {
            //shooter.update(false, false, true); // Y = FULL
            shooter.setPower(.92);
            hood.setPower(-.8);

            if (runtime.seconds() > 2) {
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
            drive.setAutoPower(-.851 * directioninalMulti,.851 * directioninalMulti,1 * directioninalMulti,-1 * directioninalMulti);
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
