package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name="Auto close Side Shoot", group="auto")
public class CloseShootAuto extends LinearOpMode {

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

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
        Hood hood = new Hood(hardwareMap);
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, lifter);
        Turret turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            directioninalMulti = 1;
            turret.limelight.pipelineSwitch(4);
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            directioninalMulti = -1;
            turret.limelight.pipelineSwitch(3);
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

        // Step 1:  Drive back for .8 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .45)) {
            shooter.update(false, true, false);    // shooter high goal
            drive.setAutoPower(-1,-1,-1,-1);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  strafe right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .65)) {
            drive.setAutoPower(1 * directioninalMulti,-1 * directioninalMulti,-1 * directioninalMulti,1 * directioninalMulti);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 3: stop
        drive.setAutoPower(0,0,0,0);

        //step 4: shoot
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8)) {
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            // start forced-fire if not already
            shooter.update(false, true, false);    // shooter high goal
            hood.setTarget(0);
            autoShoot.update();
            spin.sampleSensorsNow();
            spin.update();
            hood.update();
            autoShoot.update();
        }

        if (Parameters.upacreek == Parameters.Upacreek.NO) {
            double additive = 0;
            if (Parameters.allianceColor == Parameters.Color.RED) {
                additive = .2;
            }

                // Step 5:  strafe right for 1.3 seconds
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .4)) {
                drive.setAutoPower(.75 * directioninalMulti, -.75 * directioninalMulti, -.75 - additive * directioninalMulti, .75 + additive * directioninalMulti);
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            // Step 6: stop
            drive.setAutoPower(0, 0, 0, 0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5.25)) {
                intake.setPower(1);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(.18);
                drive.setAutoPower(.51, .51, .51, .51);
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 1:  Drive back for .8 seconds
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .8)) {
                shooter.update(false, true, false);    // shooter high goal
                drive.setAutoPower(-1, -1, -1, -1);
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  left right for 1.3 seconds
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .82)) {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
                drive.setAutoPower(-.5 * directioninalMulti, .5 * directioninalMulti, .5 * directioninalMulti, -.5 * directioninalMulti);
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            // Step 3: stop
            drive.setAutoPower(0, 0, 0, 0);

            //step 4: shoot
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .4)) {
                //turret.mode = Turret.turretMode.MANUAL;
                spin.setNeutral();
                //turret.manualControl(0.85 * directioninalMulti);
            }
            //turret.mode = Turret.turretMode.AUTO;

            //step 4: shoot
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 7)) {
                if (runtime.seconds() < 1.2) {
                    spin.goToSilo1();
                }
                turret.autoMode();
                turret.mode = Turret.turretMode.AUTO;
                // start forced-fire if not already
                shooter.update(false, true, false);    // shooter high goal
                if (runtime.seconds() > 1.3) {
                    hood.setTarget(0);

                    autoShoot.update();
                    spin.sampleSensorsNow();
                    spin.update();
                    hood.update();
                    autoShoot.update();
                }
            }

            //step 4: shoot
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2)) {
                if (runtime.seconds() < 1) {
                    spin.goToSilo1();
                } else {
                    lifter.setDown();
                }

            }
            sleep(1000);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < .8)) {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
                drive.setAutoPower(-.75 * directioninalMulti, .75 * directioninalMulti, .75 * directioninalMulti, -.75 * directioninalMulti);
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }
        else {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.2)) {
                intake.setPower(0);
                walleft.setPower(0);
                wallright.setPower(0);
                drive.setAutoPower(-.5 * directioninalMulti, .5 * directioninalMulti, .5 * directioninalMulti, -.5 * directioninalMulti);
                telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);

    }
}
