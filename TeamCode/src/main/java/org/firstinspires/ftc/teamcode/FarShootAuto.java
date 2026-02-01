package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.yise.DriveClass;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous(name = "Far Shoot Auto", group = "Auto")
public class FarShootAuto extends OpMode {
    private Paths paths;
    private int pathIndex = 0;
    private int lastPathIndex = -1;
    private static final double PATH_WAIT_SECONDS = 4.0;


    // Subsystems
    private DriveClass drive;
    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lifter;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private Turret turret;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private ElapsedTime waitTimer = new ElapsedTime();

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;


    private boolean pathJustChanged() {
        if (pathIndex != lastPathIndex) {
            lastPathIndex = pathIndex;
            return true;
        }
        return false;
    }

    public static class Paths {
        public PathChain[] paths;


        public Paths(Follower follower) {
            paths = new PathChain[14];

            paths[0] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(82.019, 8.449),
                            new Pose(127.551, 8.411)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[1] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(127.551, 8.411),
                            new Pose(73.193, 22.447)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(60)
                    )
                    .build();
            paths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(73.193, 22.447),
                            new Pose(103.798, 35.900)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[3] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.798, 35.900),
                            new Pose(130.042, 35.401)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();
            paths[4] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(130.042, 35.401),
                            new Pose(102.938, 35.748)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[5] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(102.938, 35.748),
                            new Pose(72.112, 9.047)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(60)
                    )
                    .build();

            paths[6] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(72.112, 23.739),
                            new Pose(107.899, 59.372)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[7] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(107.899, 59.372),
                            new Pose(131.367, 58.406)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[8] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131.367, 58.406),
                            new Pose(107.462, 59.457)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[9] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(107.462, 59.457),
                            new Pose(71.994, 9.297)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(60)
                    )
                    .build();

            paths[10] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(71.994, 23.442),

                            new Pose(108.034, 83.328)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[11] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(108.034, 83.328),
                            new Pose(131.011, 83.086)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[12] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(131.011, 83.086),
                            new Pose(108.328, 83.174)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();
            paths[13] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(108.328, 83.174),
                            new Pose(72.784, 9.100)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(60)
                    )
                    .build();

            // Continue mapping:
            // Path2 -> paths[2]
            // Path3 -> paths[3]
            // ...
            // Path12 -> paths[12]
        }
    }


    public void autonomousPathUpdate() {

        // Start first path
        if (pathState == 0) {
            follower.followPath(paths.paths[0]);
            pathTimer.resetTimer();
            pathState = 1;
            return;
        }

        // Normal path progression
        if (pathState == 1 && !follower.isBusy()) {

            // Wait after path finishes
            if (pathTimer.getElapsedTimeSeconds() < PATH_WAIT_SECONDS) {
                return;
            }

            pathIndex++;

            if (pathIndex < paths.paths.length) {
                follower.followPath(paths.paths[pathIndex], true);
                pathTimer.resetTimer();
            } else {
                pathState = -1; // finished
            }
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void handleShooting() {

        // ----- SAFE DEFAULTS (ALWAYS FIRST) -----
        intake.setPower(0);
        walleft.setPower(0);
        wallright.setPower(0);
        spin.setNeutral();
        shooter.update(false, false, false);

        switch (pathIndex) {
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

            case 2:
            case 6:
            case 10:
                intake.setPower(1);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(0.08);
                break;

            case 3:
            case 7:
            case 11:
                walleft.setPower(0.51);
                wallright.setPower(0.51);
                break;

            case 12:
                if(follower.getPose().getX() > 36) {
                    while (waitTimer.seconds() < 8) {
                        turret.autoMode();
                        turret.mode = Turret.turretMode.AUTO;

                        shooter.update(false, false, true);
                        hood.update();
                        autoShoot.update();
                        spin.update();
                    }
                }
                break;
        }
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        handleShooting();

        telemetry.addData("pathIndex", pathIndex);
        telemetry.addData("pathState", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(82.019, 8.449, Math.toRadians(0))
        );
        paths = new Paths(follower);


        // Hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);

        // Subsystems
        drive = new DriveClass(hardwareMap);
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        lifter = new lifter(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);

        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathIndex = 0;
        setPathState(0);
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}
