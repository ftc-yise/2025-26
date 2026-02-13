package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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

    // --- Paths ---
    public static class Paths {
        public PathChain[] paths;

        public Paths(Follower follower) {
            paths = new PathChain[4];

            // Path 13 -> 1 -> 2 -> 3 -> ... -> 12
            paths[0] = follower.pathBuilder()
                    .addPath(new BezierLine(
                                    new Pose(111.000, 136.000),

                                    new Pose(88.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(50))
                    .build();

            paths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                                    new Pose(88.000, 92.000),
                                    new Pose(135.907, 81.840),
                                    new Pose(135.991, 84.687),
                                    new Pose(135.670, 85.552),
                                    new Pose(128.078, 64.540),
                                    new Pose(118.199, 100.191),
                                    new Pose(83.031, 68.323),
                                    new Pose(82.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            paths[2] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                                    new Pose(82.000, 84.000),
                                    new Pose(111.694, 30.730),
                                    new Pose(143.143, 73.264),
                                    new Pose(143.092, 66.909),
                                    new Pose(142.278, 74.143),
                                    new Pose(84.043, 95.196)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            paths[3] = follower.pathBuilder()
                    .addPath(new BezierLine(
                                    new Pose(84.043, 95.196),

                                    new Pose(124.000, 69.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();
        }
    }

    // --- Subsystems & hardware ---
    private Follower follower;
    private Paths paths;
    private int pathIndex = 0;
    private boolean followerStarted = false;
    private Timer pathTimer, opmodeTimer;
    private DriveClass drive;
    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lifter;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private Turret turret;

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

    // Floor sensors
    private ColorSensor blc, brc, flc, frc;
    private double lastZoneTriggerStart = 0;
    private final double SENSOR_HOLD_SECONDS = 0.18;
    private final int ZONE_BLUE_THRESHOLD = 1400;

    private boolean zoneShootingActive = false;
    private double zoneShootingStartTime = 0.0;
    private final double ZONE_SHOOT_MAX_DURATION = 20;
    private final double PATH_SETTLE_SECONDS = 1.5;

    // Paths categories
    private final int[] INTAKE_PATHS = {2,3};
    private final int[] SHOOT_PATHS = {1,2,3};

    private ElapsedTime waitTimer = new ElapsedTime();

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(111,136,Math.toRadians(180)));
        paths = new Paths(follower);

        // hardware
        intake = hardwareMap.get(DcMotor.class,"intake");
        walleft = hardwareMap.get(CRServo.class,"WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class,"WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);

        // subsystems
        drive = new DriveClass(hardwareMap);
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        lifter = new lifter(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);
        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);

        // color sensors
        blc = hardwareMap.get(ColorSensor.class,"BLC");
        brc = hardwareMap.get(ColorSensor.class,"BRC");
        flc = hardwareMap.get(ColorSensor.class,"FLC");
        frc = hardwareMap.get(ColorSensor.class,"FRC");
    }

    @Override
    public void init_loop() {
        shooter.update(false,true,false);
        hood.update();
        spin.update();
        autoShoot.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathIndex = 0;
        followerStarted = false;
    }

    @Override
    public void loop() {
        // Update all subsystems
        follower.update();
        shooter.update(false,true,false);
        hood.update();
        spin.update();
        autoShoot.update();
        turret.autoMode();

        // Check color sensors for zone shooting
        checkFloorSensorsForZone();

        // Path FSM
        pathFSM();

        // Intake & wall behavior
        handleIntakeAndShooting();

        // Telemetry
        telemetry.addData("pathIndex", pathIndex);
        telemetry.addData("zoneShooting", zoneShootingActive);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addLine();
        telemetry.addData("shotsFired", autoShoot.shotsFired);
        telemetry.update();
    }

    private void pathFSM() {
        if (zoneShootingActive) return; // stay on path until shots finish

        if (!followerStarted && pathIndex < paths.paths.length) {
            follower.followPath(paths.paths[pathIndex]);
            followerStarted = true;
            pathTimer.resetTimer();
            waitTimer.reset();
            return;
        }

        // Detect path completion
        if (followerStarted && !follower.isBusy()) {
            if (pathTimer.getElapsedTimeSeconds() < PATH_SETTLE_SECONDS) return; // wait to settle

            // Shooting path triggers
            if (isInArray(SHOOT_PATHS,pathIndex)) {
                zoneShootingActive = true;
                zoneShootingStartTime = opmodeTimer.getElapsedTimeSeconds();
                spin.disableSensorUpdates();
                hood.setTarget(40);
                autoShoot.startCycle(); // forced
                return;
            }

            // Advance to next path
            pathIndex++;
            followerStarted = false;
            if (pathIndex >= paths.paths.length) pathIndex = paths.paths.length-1;
        }
    }

    private void handleIntakeAndShooting() {
        // default safe
        intake.setPower(0);
        walleft.setPower(0);
        wallright.setPower(0);
        spin.setNeutral();

        // Intake paths
        if (isInArray(INTAKE_PATHS,pathIndex)) {
            intake.setPower(1);
            walleft.setPower(1);
            wallright.setPower(1);
            spin.setManual(0.08);
        }

        // Shooting while in zone
        if (zoneShootingActive) {
            walleft.setPower(0.51);
            wallright.setPower(0.51);

            if (autoShoot.shotsFired >= 3 || opmodeTimer.getElapsedTimeSeconds()-zoneShootingStartTime > ZONE_SHOOT_MAX_DURATION) {
                zoneShootingActive = false;
                autoShoot.stopForcedCycle();
                walleft.setPower(0);
                wallright.setPower(0);
                lifter.setDown();
                spin.enableSensorUpdates();
                pathIndex++; // move on after shots
                followerStarted = false;
            }
        }
    }

    private void checkFloorSensorsForZone() {
        double rx = follower.getPose().getX();
        double ry = follower.getPose().getY();

        int count = 0;
        if (blc.blue()>ZONE_BLUE_THRESHOLD) count++;
        if (brc.blue()>ZONE_BLUE_THRESHOLD) count++;
        if (flc.blue()>ZONE_BLUE_THRESHOLD) count++;
        if (frc.blue()>ZONE_BLUE_THRESHOLD) count++;

        if (count>=3) {
            if (!zoneShootingActive) {
                zoneShootingActive = true;
                zoneShootingStartTime = opmodeTimer.getElapsedTimeSeconds();
                spin.disableSensorUpdates();
                hood.setTarget(40);
                autoShoot.startCycle();
            }
        }
    }

    private boolean isInArray(int[] arr,int val) {
        for (int v: arr) if (v==val) return true;
        return false;
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        intake.setPower(0);
        walleft.setPower(0);
        wallright.setPower(0);
        autoShoot.stopForcedCycle();
        lifter.setDown();
    }
}
