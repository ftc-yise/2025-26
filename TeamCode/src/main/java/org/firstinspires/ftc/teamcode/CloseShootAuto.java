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


@Autonomous(name = "Close Shoot Auto", group = "Auto")
public class CloseShootAuto extends OpMode {
    private Paths paths;
    private int pathIndex = 0;
    private int lastPathIndex = -1;
    private static final double PATH_WAIT_SECONDS = 4.0;

    public boolean firstTime = true;


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
    private ElapsedTime waitTimer = new ElapsedTime();

    public enum PathState{
        DRIVE_START,
        SHOOT_PRELOAD,
        BALL_LINE_3_INTAKE
    }
    PathState pathstate;
    private final Pose startPose = new Pose(83.76982892690512, 83.70139968895805,Math.toRadians(270));
    private final Pose shootPose = new Pose(130.65629860031103, 83.41524105754274,Math.toRadians(0));


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
        //public PathChain[] paths;
        private PathChain driveStartPosShootPos;
        public Paths(Follower follower) {
            driveStartPosShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose));
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            paths[1] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(83.770, 83.701),
                            new Pose(130.656, 83.415)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();
            paths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(130.656, 83.415),
                            new Pose(83.997, 83.871)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            paths[3] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(83.997, 83.871),
                            new Pose(103.860, 59.879)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();
            paths[4] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(103.860, 59.879),
                            new Pose(129.166, 59.862)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();


        }
    }


    public void autonomousPathUpdate() {

        // Start first path
        if (pathState == 0) {
            follower.followPath(paths.paths[0]);
            pathTimer.resetTimer();
            waitTimer.reset();
            pathState = 1;
            return;
        }

        // Normal path progression
        if (pathState == 1 && !follower.isBusy()) {

            // Wait after path finishes
            if (waitTimer.seconds() < PATH_WAIT_SECONDS) {
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
/*
    // Wait after path finishes
            if (pathTimer.getElapsedTimeSeconds() < PATH_WAIT_SECONDS) {
        return;
    }
*/
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

            case 0:
                intake.setPower(1);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(0.08);
                break;

            case 3:
                walleft.setPower(0.51);
                wallright.setPower(0.51);
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
                new Pose(109.456, 135.522, Math.toRadians(180))
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