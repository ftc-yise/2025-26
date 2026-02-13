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
import org.firstinspires.ftc.teamcode.yise.Ledclass;
import org.firstinspires.ftc.teamcode.yise.Parameters;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name = "Far Shoot Auto", group = "Auto")
public class FarShootAuto extends OpMode {

    // --- Paths & pathing ---
    public static class Paths {
        public PathChain[] paths;
        int X_SHIFT = 2;

        public Paths(Follower follower) {
            paths = new PathChain[3];

            // Keep earlier entries if you need them (0..2), but we start at 3 in your runtime.
            // CHANGES: split aggressive diagonals into gentler segments and add an 'approach' waypoint
            // before entering the shooting area so the follower can decelerate & line up cleanly. 82.019, 8.449
            paths[0] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                    new Pose(82.019 , 8.449),
                    new Pose(89.957 - X_SHIFT, 67.040),
                    new Pose(139.506 - X_SHIFT, 10.629),
                    new Pose(143.725 - X_SHIFT, 61.415),
                    new Pose(143.895 - X_SHIFT, 36.199),
                    new Pose(136.219 - X_SHIFT, 7.494),
                    new Pose(72.539 - X_SHIFT, 35.404),
                    new Pose(129.456 - X_SHIFT, 2.158),
                    new Pose(90 - X_SHIFT, 8.449)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            // subsequent paths unchanged but ensure their headings are small near endpoints
            paths[1] = follower.pathBuilder()
                    .addPath(new BezierCurve(
                    new Pose(90.000  - X_SHIFT, 8.449),
                    new Pose(74.175 - X_SHIFT, 83.789),
                    new Pose(73.819 - X_SHIFT, 77.214),
                    new Pose(135.934 - X_SHIFT, 29.180),
                    new Pose(142.058 - X_SHIFT, 65.776),
                    new Pose(142.328 - X_SHIFT, 73.554),
                    new Pose(141.956 - X_SHIFT, 54.422),
                    new Pose(88.713 - X_SHIFT, 55.947),
                    new Pose(82.142 - X_SHIFT, 91.552)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            paths[2] = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(82.142, 91.552), new Pose(124, 70)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

        }
    }

    // --- Enums: Auto state machine ---
    private enum AutoState {
        FOLLOW_PATH,
        REQUEST_SHOT,
        WAIT_FOR_SHOT,
        DONE
    }

    private AutoState autoState = AutoState.FOLLOW_PATH;

    // --- Configuration ---
    // Updated shooting paths per user request (we will use indices that match the
    // modified path layout). The user asked to use paths 6 and 10 for shooting.

    // fields
    private double lastZoneTriggerStart = 0.0;
    private final double SENSOR_HOLD_SECONDS = 0.18; // 180 ms

    private final int[] SHOOT_PATHS = {0, 1};      // firing-trigger paths
    private final int[] INTAKE_PATHS = {0, 1};
    private final int[] WALL_ONLY_PATHS = {4};
    private final boolean fireAtStart = true;
    private boolean startFinished = false;
    private final double PATH_SETTLE_SECONDS = 0.5; // short settle after path arrival
    private final double SHOOT_TIMEOUT_SECONDS = 20.0;
    // color threshold for "white line" detection on floor sensors (blue channel)
    private final int ZONE_BLUE_THRESHOLD = 1400;

    // --- Subsystems & hardware ---
    private Follower follower;
    private Paths paths;
    private boolean followerStarted = false;
    private boolean lastFollowerBusy = true;
    private Timer pathTimer, opmodeTimer;
    private DriveClass drive;
    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lifter;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private ShotPatternManager patternMgr;
    private Turret turret;
    private Ledclass led1;
    private Parameters parem = new Parameters();

    private DcMotor intake = null;
    private CRServo walleft = null;
    private CRServo wallright = null;

    // floor-facing color sensors at corners
    private ColorSensor BackRightCorner = null; // "BRC"
    private ColorSensor BackLeftCorner = null;  // "BLC"
    private ColorSensor FrontLeftCorner = null; // "FLC"
    private ColorSensor FrontRightCorner = null;// "FRC"

    // Local timers
    private ElapsedTime shotRequestTimer = new ElapsedTime(); // used for shot timeout / timing
    private ElapsedTime waitTimer = new ElapsedTime(); // general purpose

    // path index owned by this state machine
    // user wanted to "remove paths 0..2" -> we'll start at index 3
    private int pathIndex = -1;

    // Zone-shooting flags & latches (one per sensor)
    private boolean blcLatched = false;
    private boolean brcLatched = false;
    private boolean flcLatched = false;
    private boolean frcLatched = false;

    // When true we are firing because a floor sensor indicated we're inside a permitted zone.
    // During zone-shooting we allow movement but we won't advance to next path until shot count satisfied.
    private boolean zoneShootingActive = false;
    private double zoneShootingStartTime = 0.0;
    private final double ZONE_SHOOT_MAX_DURATION = 8; // seconds safety for zone shooting

    // helpers
    private boolean isInArray(int[] arr, int v) {
        if (arr == null) return false;
        for (int x : arr) if (x == v) return true;
        return false;
    }

    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }

    // --- Triangles for shooting zones (as requested) ---
    // Triangle 1: ((96,0),(72,24),(48,0))
    // Triangle 2: ((0,144),(72,72),(144,144))
    private boolean pointInTriangle(double px, double py, double ax, double ay, double bx, double by, double cx, double cy) {
        // barycentric technique
        double v0x = cx - ax;
        double v0y = cy - ay;
        double v1x = bx - ax;
        double v1y = by - ay;
        double v2x = px - ax;
        double v2y = py - ay;

        double dot00 = v0x * v0x + v0y * v0y;
        double dot01 = v0x * v1x + v0y * v1y;
        double dot02 = v0x * v2x + v0y * v2y;
        double dot11 = v1x * v1x + v1y * v1y;
        double dot12 = v1x * v2x + v1y * v2y;

        double denom = (dot00 * dot11 - dot01 * dot01);
        if (Math.abs(denom) < 1e-9) return false;

        double invDenom = 1.0 / denom;
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        return (u >= 0) && (v >= 0) && (u + v < 1);
    }

    private boolean insideShootingZone(double x, double y) {
        // triangle A
        if (pointInTriangle(x, y, 96+8, 0, 72+8, 24-8, 48-8, 0)) return true;
        // triangle B
        if (pointInTriangle(x, y, 0, 144, 72+8, 72+8, 144, 144)) return true;
        return false;
    }

    // ----------------- Lifecycle -----------------
    @Override
    public void init() {
        Parameters.autonomous = Parameters.AUTONOMOUS.YES;
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        // keep same starting pose but note we start at pathIndex = 3 (skipping 0..2)
        follower.setStartingPose(new Pose(82.019, 8.449, Math.toRadians(0)));
        paths = new Paths(follower);

        intake = hardwareMap.get(DcMotor.class, "intake");
        walleft = hardwareMap.get(CRServo.class, "WallWheelLeft");
        wallright = hardwareMap.get(CRServo.class, "WallWheelRight");
        led1 = new Ledclass(hardwareMap, "led1");
        wallright.setDirection(CRServo.Direction.REVERSE);

        // floor sensors (4 corners)
        BackLeftCorner = hardwareMap.get(ColorSensor.class, "BLC");
        BackRightCorner = hardwareMap.get(ColorSensor.class, "BRC");
        FrontLeftCorner = hardwareMap.get(ColorSensor.class, "FLC");
        FrontRightCorner = hardwareMap.get(ColorSensor.class, "FRC");

        drive = new DriveClass(hardwareMap);
        shooter = new ShooterClass(hardwareMap);
        spin = new Spindexer(hardwareMap);
        lifter = new lifter(hardwareMap);
        hood = new Hood(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.RED, telemetry);
        parem.autonomous = Parameters.AUTONOMOUS.YES;

        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        patternMgr = new ShotPatternManager();
        autoShoot.setPatternManager(patternMgr);
    }

    @Override
    public void init_loop() {
        // keep shooter warm for start
        shooter.update(false, false, true);

        // set limelight & pattern detection at prestart
        turret.limelight.pipelineSwitch(2);
        int tagId = turret.getID();
        ShotPatternManager.ShotPattern p = patternFromTag(tagId);
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }

        // telemetry useful prestart
        hood.update();
        Hood.TelemetryPacket H = hood.getTelemetry();
        lifter.TelemetryPacket l = lifter.getTelemetry();

        telemetry.addData("pre_pathIndex", pathIndex);
        telemetry.addData("pre_pathState", autoState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        shooter.update(false, false, true);
        opmodeTimer.resetTimer();

        // Start at index 3 to "remove" paths 0..2 as requested
        if (pathIndex < -1) pathIndex = -1;

        followerStarted = false;
        lastFollowerBusy = true;
        autoState = AutoState.FOLLOW_PATH;

        // limelight pipeline for alliance
        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            turret.limelight.pipelineSwitch(3);
        } else {
            turret.limelight.pipelineSwitch(4);
        }

        // If we want to shoot immediately before pathing, request a shot (pattern-based)
        if (fireAtStart) {
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            patternMgr.clear();
            ShotPatternManager.ShotPattern startPattern = patternFromTag(turret.getID());
            if (startPattern != null) patternMgr.addPattern(startPattern.sequence);
            autoState = AutoState.REQUEST_SHOT;

            // we will allow the state machine to handle continuation
        } else {
            // start following pathIndex immediately (we start at 3)
            if (pathIndex < paths.paths.length) {
                follower.followPath(paths.paths[pathIndex]);
                followerStarted = true;
            }
        }

        // reset timers
        shotRequestTimer.reset();
        waitTimer.reset();
    }

    @Override
    public void loop() {
        // --- Single authoritative updates for subsystems ---
        autoShoot.update();     // ONLY once per loop
        hood.update();
        spin.sampleSensorsNow();
        spin.update();
        shooter.updateTelemetry();
        follower.update();

        // check floor sensors and potentially trigger zone-shooting
        checkFloorSensorsForZoneAndMaybeStartShooting();

        // run single owner auto state machine
        autoStateUpdate();

        if (startFinished) {
            // Intake/wall logic for non-shooting behavior while following paths
            if (autoState == AutoState.FOLLOW_PATH) {
                if (isInArray(INTAKE_PATHS, pathIndex)) {
                    intake.setPower(1);
                    walleft.setPower(1);
                    wallright.setPower(1);
                    spin.setManual(0.08);
                    turret.stop();
                } else if (isInArray(WALL_ONLY_PATHS, pathIndex)) {
                    intake.setPower(0);
                    walleft.setPower(0.51);
                    wallright.setPower(0.51);
                } else {
                    // default
                    intake.setPower(0);
                    walleft.setPower(0);
                    wallright.setPower(0);
                }
            }

            // If zone-shooting active, keep shooter/spindexer/turret updated and do not advance path index
            if (zoneShootingActive) {
                hood.update();
                autoShoot.update(); // ensure firing continues
                spin.update();
                walleft.setPower(0.51);
                wallright.setPower(0.51);

                // If we've fired all 3 balls, stop zone-shooting and allow path advancement
                if (autoShoot.shotsFired >= 3) {
                    zoneShootingActive = false;
                    stopZoneShootingCleanup();
                } else if ((opmodeTimer.getElapsedTimeSeconds() - zoneShootingStartTime) > ZONE_SHOOT_MAX_DURATION) {
                    // safety timeout - abort zone shooting
                    zoneShootingActive = false;
                    autoShoot.stopForcedCycle();
                    stopZoneShootingCleanup();
                }
            }
        }

            // --- Telemetry (condensed, but thorough) ---
            Hood.TelemetryPacket H = hood.getTelemetry();
            lifter.TelemetryPacket l = lifter.getTelemetry();

            telemetry.addData("AutoState", autoState);
            telemetry.addData("pathIndex", pathIndex);
            telemetry.addData("followerBusy", follower.isBusy());
            telemetry.addData("zoneShooting", zoneShootingActive);
            telemetry.addData("shotsFired", autoShoot.shotsFired);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addLine();

            ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", s.mode);
            telemetry.addData("Target RPM", "%.2f", s.targetRPM);
            telemetry.addData("Current RPM", "%.1f", s.currentRPM);
            telemetry.addData("Error RPM", "%.1f", s.errorRPM);
            telemetry.addData("volt", s.motorPower);

            telemetry.addLine("=== SPINDEXER ===");
            Spindexer.TelemetryPacket spina = spin.getTelemetry();
            telemetry.addData("Mode", spina.mode);
            telemetry.addData("Angle", "%.1f°", spina.currentAngle);
            telemetry.addData("Target", "%.1f°", spina.targetAngle);
            telemetry.addData("Error", "%.1f°", spina.angleError);
            telemetry.addData("Power", "%.2f", spina.appliedPower);

            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Mode", turret.mode);
            telemetry.addData("pose", turret.getPose());
            telemetry.addData("id", turret.getID());
            telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

            telemetry.addLine("=== HOOD ===");
            telemetry.addData("Mode", H.mode);
            telemetry.addData("Angle", "%.1f°", H.currentAngle);
            telemetry.addData("Target", "%.1f°", H.targetAngle);
            telemetry.addData("Error", "%.1f°", H.angleError);

            telemetry.update();
    }

    private void stopZoneShootingCleanup() {
        // Stop feeding and restore normal behavior so state machine can advance
        walleft.setPower(0);
        wallright.setPower(0);
        lifter.setDown();
        // re-enable spindexer sensor updates if they were disabled by forced cycle
        spin.enableSensorUpdates();
    }

    @Override
    public void stop() {
        // clean up if desired
    }

    // ----------------- Auto state machine -----------------
    private void autoStateUpdate() {

        // If zone-shooting active, prevent normal path advancement. We still update follower, allowing drive motion.
        if (zoneShootingActive) {
            // keep current pathIndex until zone shooting completes
            return;
        }

        switch (autoState) {

            case FOLLOW_PATH:
                // Start path if not yet started
                startFinished = true;
                if (!followerStarted) {
                    if (pathIndex < paths.paths.length) {
                        follower.followPath(paths.paths[pathIndex], 0.8, false);
                        followerStarted = true;
                        // ensure lastFollowerBusy is true until follower reports no longer busy
                        lastFollowerBusy = true;
                    } else {
                        autoState = AutoState.DONE;
                        return;
                    }
                }

                // Detect arrival edge: follower busy -> not busy
                if (lastFollowerBusy && !follower.isBusy()) {
                    // arrived at path endpoint, start settle timer
                    pathTimer.resetTimer();
                    lastFollowerBusy = false;
                }

                // while follower is busy keep waiting
                if (follower.isBusy()) {
                    lastFollowerBusy = true;
                    return;
                }

                // Wait a little to settle after arrival
                if (pathTimer.getElapsedTimeSeconds() < PATH_SETTLE_SECONDS) {
                    return;
                }

                // If this path is a shooting path, request shooting (pattern-based)
                if (isInArray(SHOOT_PATHS, pathIndex)) {
                    autoState = AutoState.REQUEST_SHOT;
                    return;
                }

                // Not a shooting path -> advance to next path
                pathIndex++;
                followerStarted = false;

                if (pathIndex >= paths.paths.length) {
                    autoState = AutoState.DONE;
                }
                break;

            case REQUEST_SHOT:
                // Prepare turret and pattern manager

                // Load pattern based on tag if present (keeps behavior consistent with init_loop)
                ShotPatternManager.ShotPattern p = patternFromTag(turret.getID());
                if (p != null) {
                    patternMgr.clear();
                    patternMgr.addPattern(p.sequence);
                }

                // start shooter job (pattern-aware) — use startCycle so pattern manager is honored
                autoShoot.startCycle();
                // break follower control to keep robot still while pattern-based shooting occurs
                // (BUT the user also wanted optionally to shoot while moving when in a zone - zone shooting uses forced cycle.)
                if (!zoneShootingActive) {
                    follower.breakFollowing();
                    followerStarted = false;
                }
                // start shot timeout timer
                shotRequestTimer.reset();
                autoState = AutoState.WAIT_FOR_SHOT;
                break;

            case WAIT_FOR_SHOT:
                // maintain turret auto alignment while waiting/ firing
                hood.setTarget(40);

                // keep walls on while shooting
                walleft.setPower(0.51);
                wallright.setPower(0.51);

                // If shooter finished (job is done) -> proceed
                if (autoShoot.shotsFired >= 3) {
                    // stop feeding & reset actuators
                    walleft.setPower(0);
                    wallright.setPower(0);
                    lifter.setDown();

                    pathIndex++;
                    followerStarted = false;

                    // short reset of timers for next path
                    pathTimer.resetTimer();
                    waitTimer.reset();

                    if (pathIndex >= paths.paths.length) {
                        autoState = AutoState.DONE;
                    } else {
                        autoState = AutoState.FOLLOW_PATH;
                    }
                    startFinished = true;
                    return;
                }

                // safety timeout: force stop after SHOOT_TIMEOUT_SECONDS
                if (shotRequestTimer.seconds() > SHOOT_TIMEOUT_SECONDS) {
                    // attempt a graceful stop
                    autoShoot.stopForcedCycle(); // safe; if not forced this will move to COMPLETE eventually
                    walleft.setPower(0);
                    wallright.setPower(0);
                    lifter.setDown();

                    pathIndex++;
                    followerStarted = false;
                    if (pathIndex >= paths.paths.length) autoState = AutoState.DONE;
                    else autoState = AutoState.FOLLOW_PATH;
                    startFinished = true;
                }
                break;

            case DONE:
                // finished all paths
                follower.breakFollowing();
                // ensure systems in safe states
                walleft.setPower(0);
                wallright.setPower(0);
                lifter.setDown();
                break;
        }

    }

    // ----------------- Zone detection and zone-shooting -----------------
    private void checkFloorSensorsForZoneAndMaybeStartShooting() {
        int blc = BackLeftCorner.blue();
        int brc = BackRightCorner.blue();
        int flc = FrontLeftCorner.blue();
        int frc = FrontRightCorner.blue();

        double rx = follower.getPose().getX();
        double ry = follower.getPose().getY();
        boolean coarseInside = insideShootingZone(rx, ry);

        if (!coarseInside) {
            // reset hold timer & latches if robot not in coarse zone
            lastZoneTriggerStart = 0;
            blcLatched = brcLatched = flcLatched = frcLatched = false;
            return;
        }

        int countAbove = 0;
        if (blc > ZONE_BLUE_THRESHOLD) countAbove++;
        if (brc > ZONE_BLUE_THRESHOLD) countAbove++;
        if (flc > ZONE_BLUE_THRESHOLD) countAbove++;
        if (frc > ZONE_BLUE_THRESHOLD) countAbove++;

        // immediate accept if two or more sensors are high (quick entry)
        if (countAbove >= 2) {
            startZoneShootingIfNeeded();
            return;
        }

        // otherwise require a single sensor held high for SENSOR_HOLD_SECONDS
        boolean anyHigh = (blc > ZONE_BLUE_THRESHOLD) || (brc > ZONE_BLUE_THRESHOLD)
                || (flc > ZONE_BLUE_THRESHOLD) || (frc > ZONE_BLUE_THRESHOLD);

        double now = opmodeTimer.getElapsedTimeSeconds();
        if (anyHigh) {
            if (lastZoneTriggerStart == 0) lastZoneTriggerStart = now;
            else if ((now - lastZoneTriggerStart) >= SENSOR_HOLD_SECONDS) {
                startZoneShootingIfNeeded();
                lastZoneTriggerStart = 0;
            }
        } else {
            lastZoneTriggerStart = 0;
        }

        // clear latches on exit (keep for debug if you like)
        blcLatched = blc > ZONE_BLUE_THRESHOLD;
        brcLatched = brc > ZONE_BLUE_THRESHOLD;
        flcLatched = flc > ZONE_BLUE_THRESHOLD;
        frcLatched = frc > ZONE_BLUE_THRESHOLD;
    }

    private void startZoneShootingIfNeeded() {
        // do nothing if already in zone shooting
        if (zoneShootingActive) return;

        // Begin forced shooting so shooter can feed while robot continues to move.
        // We will *not* allow the state machine to advance pathIndex while zoneShootingActive == true.
        zoneShootingActive = true;
        zoneShootingStartTime = opmodeTimer.getElapsedTimeSeconds();
        // Disable spindexer sensor updates so forced cycle feeding isn't interrupted
        spin.disableSensorUpdates();
        // Start forced mode so feeding occurs while follower updates keep running
        autoShoot.startCycle(); // this sets forceShooting=true and allows continuous cycling
        // Note: we'll stop the forced cycle when shotsFired >= 3 (or timeout)
    }
}
