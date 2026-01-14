package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pedro Segmented Auto — revised:
 * - AprilTag -> choose shot pattern (uses turret.getID())
 * - Ball-color sequencing for shooting
 * - Safety timeouts (path + per-shot + overall)
 * - Telemetry overlays for tuning
 *
 * This file is intentionally conservative: it uses your existing class APIs
 * (ShooterClass, Spindexer, ShooterExecutionClass, Turret, Pedro follower).
 */
@Autonomous(name = "Close Auto", group = "auto")
public class CloseShootAuto extends LinearOpMode {

    private Turret turret;
    private Spindexer spin;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private lifter lifter;
    private ShooterClass shooter;

    // Your Turret class provides getID() (returns tag id or -1).
    public int id = -1;




    // follower + paths
    private Follower follower;
    private Paths paths;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime overallTimer = new ElapsedTime();

    // hardware
    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

    // state machine
    private int pathState = 0;

    // safety/timeouts (tweakable)
    private static final double PATH_TIMEOUT = 6.0;    // seconds per path
    private static final double SHOT_TIMEOUT = 3.0;    // seconds per shot attempt
    private static final double AUTO_TIMEOUT = 30.0;   // overall auto timeout

    // shooting pattern (filled by AprilTag detection)
    private enum ShotColor { GREEN, PURPLE }
    private ShotColor[] shotPattern = new ShotColor[]{ ShotColor.PURPLE, ShotColor.PURPLE, ShotColor.GREEN };
    private int shotIndex = 0;
    org.firstinspires.ftc.teamcode.yise.Turret.turretAlliance alliance = org.firstinspires.ftc.teamcode.yise.Turret.turretAlliance.BLUE;

    @Override
    public void runOpMode() {
        // ---------------- init subsystems (use your existing names)
        this.shooter = new ShooterClass(hardwareMap);
        this.spin = new Spindexer(hardwareMap);
        this.hood = new Hood(hardwareMap);
        this.lifter = new lifter(hardwareMap);
        this.autoShoot = new ShooterExecutionClass(this.spin, this.shooter, hardwareMap, this.lifter);
        this.turret = new Turret(hardwareMap, alliance, telemetry);

        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        wallright.setDirection(CRServo.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");

        follower = Constants.createFollower(hardwareMap);

        // poses (your original points)
        Pose p0 = new Pose(18.0, 132.0);
        Pose p1 = new Pose(60.0, 96.0 + 12);
        Pose p2 = new Pose(36.0, 84.0);

        follower.setStartingPose(new Pose(p0.getX(), p0.getY(), Math.toRadians(315)));
        paths = new Paths(follower, p0, p1, p2);

        // detect AprilTag, set shotPattern
        detectPatternFromLimelight();

        telemetry.addData("Status", "Init done");
        telemetry.addData("pattern", id);
        telemetry.update();



        waitForStart();
        stateTimer.reset();
        overallTimer.reset();

        // detect AprilTag, set shotPattern
        detectPatternFromLimelight();

        // small delay to let everything boot
        sleep(160);

        // main loop
        while (opModeIsActive() && overallTimer.seconds() < AUTO_TIMEOUT) {
            telemetry.addData("INIT CHECK",
                    shooter != null && spin != null && autoShoot != null && turret != null);
            telemetry.update();

            // core updates (non-blocking)
            follower.update();
            shooter.update(false, true, false);   // using your "low" preset
            autoShoot.update();
            spin.update();
            turret.autoMode();  // your turret class provided autoMode earlier

            // state machine
            autonomousPathUpdate();

            // telemetry
            telemetryOverlay();

            // small sleep to be friendly with CPU
            sleep(10);
        }

        // ensure motors off
        intake.setPower(0);
        hood.stop();
        shooter.stop();
        spin.stop();
    }

    // ---------------- state machine
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.path1);
                stateTimer.reset();
                pathState = 1;
                break;

            case 1: // wait for path1 complete or timeout
                if (!follower.isBusy() || stateTimer.seconds() > PATH_TIMEOUT) {
                    // prepare to shoot according to detected pattern
                    shotIndex = 0;
                    stateTimer.reset();
                    pathState = 2;
                }
                break;

            case 2: // shoot pattern
                if (shotIndex >= shotPattern.length) {
                    // finished pattern -> small wait then continue
                    stateTimer.reset();
                    pathState = 3;
                    break;
                }

                // find a silo that matches the desired color
                int siloIdx = findSiloForColor(shotPattern[shotIndex]);
                if (siloIdx == -1) {
                    // no matching color found (maybe already empty) -> skip
                    shotIndex++;
                    stateTimer.reset();
                    break;
                }

                while (stateTimer.seconds() < 2.00){
                    turret.runto(670);
                }
                if (stateTimer.seconds() > 2.3) {
                    stateTimer.reset();
                }

                // command spindexer to that silo
                switch (siloIdx) {
                    case 0: spin.goToSilo1(); break;
                    case 1: spin.goToSilo2(); break;
                    case 2: spin.goToSilo3(); break;
                }

                // wait for alignment AND shooter ready
                boolean aligned = Math.abs(spin.getTelemetry().angleError) < 3.0;
                boolean shooterReady = shooter.getTelemetry().readyNormal; // your ShooterClass exposes this
                if (aligned && shooterReady) {
                    if (!autoShoot.isBusy()) {
                        autoShoot.startCycle();
                    }
                }

                // once shot cycle completes, advance
                if (!autoShoot.isBusy() && aligned) {
                    shotIndex++;
                    stateTimer.reset();
                }

                // safety: give up on this shot after SHOT_TIMEOUT
                if (stateTimer.seconds() > SHOT_TIMEOUT) {
                    shotIndex++;
                    stateTimer.reset();
                }
                break;

            case 99:
                // finished
                break;

            default:
                break;
        }
    }

    // ---------------- AprilTag detection using your turret/limelight helper
    private void detectPatternFromLimelight() {
        try {
            id = turret.getID();
        } catch (Exception e) {
            // ignore; fallback
            id = -1;
        }

        if (id == 21) {
            shotPattern = new ShotColor[]{ ShotColor.GREEN, ShotColor.PURPLE, ShotColor.PURPLE };
        } else if (id == 22) {
            shotPattern = new ShotColor[]{ ShotColor.PURPLE, ShotColor.GREEN, ShotColor.PURPLE };
        } else if (id == 23) {
            shotPattern = new ShotColor[]{ ShotColor.PURPLE, ShotColor.PURPLE, ShotColor.GREEN };
        } else {
            // fallback default
            shotPattern = new ShotColor[]{ ShotColor.PURPLE, ShotColor.PURPLE, ShotColor.GREEN };
        }
    }

    // ---------------- helper to find which silo currently contains a given color
    private int findSiloForColor(ShotColor color) {
        Spindexer.BallColor[] silos = spin.getTelemetry().siloColors;
        for (int i = 0; i < silos.length; i++) {
            if (color == ShotColor.GREEN && silos[i] == Spindexer.BallColor.GREEN) return i;
            if (color == ShotColor.PURPLE && silos[i] == Spindexer.BallColor.PURPLE) return i;
        }
        return -1;
    }

    // ---------------- telemetry for tuning / visibility
    private void telemetryOverlay() {
        telemetry.addData("state", pathState);
        telemetry.addData("overallSec", String.format("%.1f", overallTimer.seconds()));
        telemetry.addData("shotIndex", shotIndex);
        telemetry.addData("pattern", shotPattern[0] + " " + shotPattern[1] + " " + shotPattern[2]);
        telemetry.addData("shooterRPM", "%.0f", shooter.getTelemetry().currentRPM);
        telemetry.addData("shooterErr", "%.0f", shooter.getTelemetry().errorRPM);
        telemetry.addData("shooterReady", shooter.getTelemetry().readyNormal);
        telemetry.addData("spindexerErr", "%.2f", spin.getTelemetry().angleError);
        telemetry.addData("pathTimer", "%.2f", stateTimer.seconds());
        telemetry.update();
    }

    // ---------------- Paths
    public static class Paths {
        public PathChain path1, path2, path3;

        public Paths(Follower follower, Pose p0, Pose p1, Pose p2) {

            path1 = follower.pathBuilder()
                    .addPath(new BezierCurve(new Pose(p0.getX(), p0.getY()), new Pose(p1.getX(), p1.getY()), new Pose(p2.getX(), p2.getY())))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
