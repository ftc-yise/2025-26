package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name="Auto close Side Shoot", group="auto")
public class CloseShootAuto extends LinearOpMode {
    private CRServo hood = null;
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
        ShooterExecutionClass autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        Turret turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            directioninalMulti = 1;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            directioninalMulti = -1;
        }

        hood = hardwareMap.get(CRServo.class, "hood");

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
        while (opModeIsActive() && (runtime.seconds() < .5)) {
            drive.setAutoPower(-1,-1,-1,-1);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  strafe right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .75)) {
            drive.setAutoPower(1 * directioninalMulti,-1 * directioninalMulti,-1 * directioninalMulti,1 * directioninalMulti);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 3: stop
        drive.setAutoPower(0,0,0,0);

        //step 4: shoot
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 8)) {
            shooter.update(false, true, false); // Y = FULL
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

        // Step 5:  strafe right for 1.3 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .8)) {
            drive.setAutoPower(1 * directioninalMulti,-1 * directioninalMulti,-1 * directioninalMulti,1 * directioninalMulti);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 6: stop
        drive.setAutoPower(0,0,0,0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            intake.setPower(.6);
            walleft.setPower(1);
            wallright.setPower(1);
            spin.setManual(.1);
            drive.setAutoPower(1,1,1,1);
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
