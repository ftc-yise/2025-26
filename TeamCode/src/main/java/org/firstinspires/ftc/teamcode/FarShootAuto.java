package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name="Auto Far Side Shoot", group="auto")
public class FarShootAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static double     directioninalMulti    = 1;
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;


    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;


    @Override
    public void runOpMode() {

        DriveClass drive = new DriveClass(hardwareMap);
        ShooterClass shooter = new ShooterClass(hardwareMap);
        Spindexer spin = new Spindexer(hardwareMap);
        lifter lifter = new lifter(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, lifter);
        Turret turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            directioninalMulti = 1;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            directioninalMulti = -1;
        }

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);


        intake = hardwareMap.get(DcMotor.class, "intake");

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
        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            directioninalMulti = 1;
            turret.limelight.pipelineSwitch(4);
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            directioninalMulti = -1;
            turret.limelight.pipelineSwitch(3);
        }
        turret.autoMode();
        runtime.reset();
        while (opModeIsActive() && ((runtime.seconds() < 12))) {
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            // start forced-fire if not already
            shooter.update(false, false, true);    // shooter high goal
            if (runtime.seconds() > 3) {
                hood.setTarget(0);

                autoShoot.update();
                spin.sampleSensorsNow();
                spin.update();
                hood.update();
                autoShoot.update();
                turret.autoMode();
            }
        }

        // Step 2:  Spin right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.1)) {
            lifter.setDown();
            if (Parameters.allianceColor == Parameters.Color.BLUE) {
                drive.setAutoPower(1, 0, 0, 1);
            }else {
                drive.setAutoPower(-0, 1, 1, -0);
            }
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 4:  Stop
        drive.setAutoPower(0,0,0,0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.25)) {
            intake.setPower(1);
            walleft.setPower(1);
            wallright.setPower(1);
            spin.setManual(.08);
            drive.setAutoPower(.51, .51, .51, .51);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            shooter.update(false, false, false);    // shooter high goal
        }

            runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.25)) {

            spin.setNeutral();
            drive.setAutoPower(-.51, -.51, -.51, -.51);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .7)) {
            intake.setPower(0);
            if (Parameters.allianceColor == Parameters.Color.BLUE) {
                drive.setAutoPower(1, 0, 0, 1);
            }else {
                drive.setAutoPower(-0, -.8, -.8, -0);
            }
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        drive.setAutoPower(0,0,0,0);


        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 9)) {
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            //spin.setNeutral();
            // start forced-fire if not already
            shooter.update(false, false, true);    // shooter high goal
                hood.setTarget(0);
                autoShoot.update();
                spin.sampleSensorsNow();
                spin.update();
                hood.update();
                autoShoot.update();
                turret.autoMode();
        }

        //step 4: shoot
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            if (runtime.seconds() < 2) {
                spin.goToSilo1();
            } else {
                lifter.setDown();
                drive.setAutoPower(.51, .51, .51, .51);
            }

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
