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
    private static final double PATH_WAIT_SECONDS = 1.0;

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

    private int pathState;

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
            paths = new PathChain[13];

            paths[0] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(109.456, 135.522),
                            new Pose(90.661, 120.478)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(38)
                    )
                    .build();

            paths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(90.661, 120.478),
                            new Pose(73.117, 84.132),
                            new Pose(95.367, 85.166)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(38),
                            Math.toRadians(-6)
                    )
                    .build();
            paths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(95.367, 85.166),
                            new Pose(129.895, 83.565)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[3] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(129.895, 83.565),
                            new Pose(106.517, 83.565)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();
            paths[4] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(106.517, 83.565),
                            new Pose(85.404, 87.064),
                            new Pose(71.002, 91.923)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[5] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(71.002, 91.923),
                            new Pose(70.817, 60.918),
                            new Pose(104.145, 60.257)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[6] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(104.145, 60.257),

                            new Pose(130.245, 59.552)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[7] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(130.245, 59.552),

                            new Pose(103.807, 60.132)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[8] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(103.807, 60.132),
                            new Pose(72.234, 60.400),
                            new Pose(70.822, 92.458)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[9] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(70.822, 92.458),
                            new Pose(50.768, 36.534),
                            new Pose(103.124, 35.824)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[10] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(103.124, 35.824),

                            new Pose(130.478, 35.110)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[11] = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(130.478, 35.110),

                            new Pose(103.356, 35.460)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(-6)
                    )
                    .build();

            paths[12] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(103.356, 35.460),
                            new Pose(48.962, 37.448),
                            new Pose(70.585, 92.306)

                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-6),
                            Math.toRadians(38)
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
                if (firstTime){
                    autoShoot.startForcedCycle();
                }
                firstTime = false;
                turret.autoMode();
                turret.mode = Turret.turretMode.AUTO;

                shooter.update(false, false, true);
                hood.update();
                autoShoot.update();
                spin.update();
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
