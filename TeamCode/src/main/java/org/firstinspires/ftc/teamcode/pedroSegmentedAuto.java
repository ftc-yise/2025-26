package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;

@Autonomous(name = "Pedro Segmented Auto (scaled)", group = "auto")
public class pedroSegmentedAuto extends LinearOpMode {
    // ---- make these class fields (NOT local to runOpMode) ----
    private ShooterClass shooter;
    private Spindexer spin;
    private ShooterExecutionClass autoShoot;
    private Turret turret;

    // Pedro follower + paths
    private Follower follower;
    private Paths paths;
    private int pathState = 0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();


    // Example hardware you asked for (adjust names to match your config)
    private CRServo hood = null;
    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;
    private int directioninalMulti;
    // SENSOR SCALE: problem: when real = 24" sensor reads 15"
    // scaleFactor = real / reading = 24 / 15 = 1.6
    // We apply this scale to all X/Y pose coordinates (scaling from origin).
    private static final double SCALE_FACTOR = (15.0 / 24.0);


    org.firstinspires.ftc.teamcode.yise.Turret.turretAlliance alliance = Turret.turretAlliance.BLUE;

    @Override
    public void runOpMode() {
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.BLUE, telemetry);

        if (Parameters.allianceColor == Parameters.Color.RED) {
            alliance = Turret.turretAlliance.RED;
            directioninalMulti = -1;
        } else if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            directioninalMulti = 1;
        }

        // --- hardware init (adjust keys to your map) ---
        hood = hardwareMap.get(CRServo.class, "hood");

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);


        intake = hardwareMap.get(DcMotor.class, "intake");
        // --- Pedro follower init ---
        follower = Constants.createFollower(hardwareMap);

        // Convert your list of poses into Pose objects (no headings given).
        // Original poses:
        // (18.0, 132.0), (60.0, 96.0), (36.0, 84.0), (12.0, 84.0),
        // (64.0, 72.0), (36.0, 60.0), (12.0, 60.0), (63.729, 8.206)
        //
        // We scale each point by SCALE_FACTOR to compensate for sensor bias.
        Pose p0 = scaledPose(18.0, 132.0);
        Pose p1 = scaledPose(60.0, 96.0);
        Pose p2 = scaledPose(36.0, 84.0);
        Pose p3 = scaledPose(12.0, 84.0);
        Pose p4 = scaledPose(64.0, 72.0);
        Pose p5 = scaledPose(36.0, 60.0);
        Pose p6 = scaledPose(12.0, 60.0);
        Pose p7 = scaledPose(63.729, 8.206);

        // set starting pose to the scaled first pose and a heading pointing toward p1
        //double startHeading = headingToward(p0, p1);
        double startHeading = Math.toRadians(-135);;

        follower.setStartingPose(new Pose(p0.getX(), p0.getY(), startHeading));

        // Build segmented Paths (so we can stop and run actions between segments).
        paths = new Paths(follower, p0, p1, p2, p3, p4, p5, p6, p7);

        telemetry.addData("Status", "Initialized - scale=" + SCALE_FACTOR);
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        // small initial delay (optional)
        sleep(100);

        // Main loop: keep updating follower and the state machine
        while (opModeIsActive()) {
            follower.update(); // REQUIRED: updates internal follower state
            autonomousPathUpdate(); // state machine drives followPath calls and actions

            // Telemetry (quick)
            telemetry.addData("state", pathState);
            telemetry.addData("pose", String.format("%.1f, %.1f", follower.getPose().getX(), follower.getPose().getY()));
            telemetry.update();

            // short idle to avoid hogging CPU
            sleep(10);
        }
    }

    // --- Helper: scale a raw x,y into a Pose (keeps heading at 0 for now) ---
    private Pose scaledPose(double x, double y) {
        return new Pose(x * SCALE_FACTOR, y * SCALE_FACTOR);
    }

    // Helper: compute heading from a -> b (radians)
    private double headingToward(Pose a, Pose b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Very simple state machine:
     *
     * 0 -> start segment1 (p0 -> p1 -> p2)
     * 1 -> wait for follower done, run "arm action" for 1s
     * 2 -> start segment2 (p2 -> p3 -> p4)
     * 3 -> wait for follower done, run intake action for 1.2s
     * 4 -> start segment3 (p4 -> p5 -> p6 -> p7)
     * 5 -> done / park (stop motors)
     */
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start path 1
                follower.followPath(paths.path1);
                pathState = 1;
                stateTimer.reset();
                break;

            case 1:
                // Wait until follower finishes path1
                if (!follower.isBusy()) {
                    // Run an example arm action for 1 second (servo move)
                    stateTimer.reset();
                    runtime.reset();
                    pathState = 11; // transitional substate to run the timed action
                }
                break;

                // Substate: run arm action for 1.0s then continue
            case 11:
                shooter.update(false, true, false);
                hood.setPower(-1);

                if (runtime.seconds() > 3) {
                    if (!autoShoot.isBusy()) {
                        autoShoot.startCycle();
                    }
                    autoShoot.update();
                    spin.update();
                    turret.autoMode();
                }

                if (runtime.seconds() > 8) {
                    hood.setPower(0);
                    pathState = 2;
                }
                break;

            case 2:
                // Start path 2
                follower.followPath(paths.path2);
                pathState = 3;
                stateTimer.reset();
                break;

            case 3:
                // Wait for path 2 to complete, then run intake example
                if (!follower.isBusy()) {
                    // Example intake run for 1.2s
                    intake.setPower(0.6);
                    stateTimer.reset();
                    pathState = 31;
                }
                break;

            case 31:
                if (stateTimer.seconds() > 1.2) {
                    intake.setPower(0);
                    sleep(50);
                    pathState = 4;
                }
                break;

            case 4:
                // Start the final path (long segment to p7)
                follower.followPath(paths.path3);
                pathState = 5;
                break;

            case 5:
                // Wait until final path completes
                if (!follower.isBusy()) {
                    // We are parked — stop any motors/actuators and finish
                    intake.setPower(0);

                    pathState = 99;
                }
                break;

            case 99:
                // Completed. Keep things idle
                break;

            default:
                // fallback
                break;
        }
    }

    // --- Paths: construct 3 segment PathChain objects using your poses ---
    public static class Paths {
        public PathChain path1;
        public PathChain path2;
        public PathChain path3;

        public Paths(Follower follower, Pose p0, Pose p1, Pose p2, Pose p3, Pose p4, Pose p5, Pose p6, Pose p7) {

            // Segment 1: p0 -> p1 -> p2
            path1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(p0.getX(), p0.getY()),
                                    new Pose(p1.getX(), p1.getY()),
                                    new Pose(p2.getX(), p2.getY())
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // Segment 2: p2 -> p3 -> p4
            path2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(p2.getX(), p2.getY()),
                                    new Pose(p3.getX(), p3.getY()),
                                    new Pose(p4.getX(), p4.getY())
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // Segment 3: p4 -> p5 -> p6 -> p7
            path3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(p4.getX(), p4.getY()),
                                    new Pose(p5.getX(), p5.getY()),
                                    new Pose(p6.getX(), p6.getY()),
                                    new Pose(p7.getX(), p7.getY())
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
