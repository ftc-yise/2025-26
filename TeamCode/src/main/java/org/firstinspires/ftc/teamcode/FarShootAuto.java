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
import org.firstinspires.ftc.teamcode.yise.ShotPatternManager;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name = "Far Shoot Auto", group = "Auto")
public class FarShootAuto extends OpMode {
    private Paths paths;
    private int pathIndex = 0;
    private int lastPathIndex = -1;
    private boolean shootingActive = false;
    private static final double PATH_WAIT_SECONDS = 4.0;
    public boolean firstTime = true;
    // Add these fields:
    private boolean lastFollowerBusy = true;
    // NOTE: removed 0 from shoot paths to avoid double-shoot when fireAtStart == true
    private final int[] SHOOT_PATHS = {5, 9, 13};
    private final int[] INTAKE_PATHS = {3, 7, 11};
    private final int[] WALL_ONLY_PATHS = {4, 8, 12};
    private final double SHOOT_TIMEOUT = 8.0;          // seconds
    // If you want shooting to happen before the first path, set true:
    private final boolean fireAtStart = true;
    private boolean followerStarted = false;

    // Subsystems
    private DriveClass drive;
    private ShooterClass shooter;
    private Spindexer spin;
    private lifter lifter;
    private Hood hood;
    private ShooterExecutionClass autoShoot;
    private ShotPatternManager patternMgr;
    Turret.turretAlliance alliance = Turret.turretAlliance.RED;

    private ShotPatternManager.ShotPattern patternFromTag(int tagId) {
        switch (tagId) {
            case 21: return ShotPatternManager.ShotPattern.GPP;
            case 22: return ShotPatternManager.ShotPattern.PGP;
            case 23: return ShotPatternManager.ShotPattern.PPG;
            default: return null;
        }
    }

    private Turret turret;
    private Parameters parem;

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
                            new Pose(72.112, 23.739)
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
                            new Pose(71.994, 23.442)
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
                            new Pose(72.784, 23.291)
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
        // If path system not started yet and we are not shooting at start -> start it
        if (!followerStarted && !shootingActive && pathState == 0) {
            follower.followPath(paths.paths[pathIndex]);
            followerStarted = true;
            pathTimer.resetTimer();
            pathState = 1;
            return;
        }

        // Only do normal progression when we're not shooting
        if (!follower.isBusy() && !shootingActive) {

            // small wait after path arrival
            if (pathTimer.getElapsedTimeSeconds() < PATH_WAIT_SECONDS) {
                firstTime = true;
                return;
            }

            // If current path is a shooting-path, start shooting instead of advancing
            for (int p : SHOOT_PATHS) {
                if (p == pathIndex) {
                    shootingActive = true;
                    waitTimer.reset();
                    spin.setNeutral();
                    // Use pattern-aware cycle so ShotPatternManager is respected
                    patternMgr.clear();
                    patternMgr.addPattern(patternFromTag(turret.getID()).sequence);
                    autoShoot.startCycle();
                    follower.breakFollowing(); // hold robot while shooting
                    followerStarted = false;
                    return; // do not increment pathIndex — handleShooting will advance when done
                }
            }

            // Not a shooting path — advance to next path
            pathIndex++;
            if (pathIndex < paths.paths.length) {
                follower.followPath(paths.paths[pathIndex], false);
                followerStarted = true;
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
        // SAFE DEFAULTS
        //intake.setPower(0);
        //walleft.setPower(0);
        //wallright.setPower(0);
        //spin.setNeutral();
        //shooter.update(false, false, false);

        // Intake/wall behavior (non-shooting paths)
        for (int p : INTAKE_PATHS) {
            if (pathIndex == p) {
                intake.setPower(1);
                walleft.setPower(1);
                wallright.setPower(1);
                spin.setManual(0.08);
                break;
            }
        }
        for (int p : WALL_ONLY_PATHS) {
            if (pathIndex == p) {
                walleft.setPower(0.51);
                wallright.setPower(0.51);
                break;
            }
        }

        // Shooting active loop
        if (shootingActive) {
            // keep shooter/spindexer/lift alive
            hood.update();
            autoShoot.update();
            spin.update();
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;

            // walls on during shooting
            walleft.setPower(0.51);
            wallright.setPower(0.51);

            // stop shooting when done or timeout
            if (waitTimer.seconds() >= SHOOT_TIMEOUT) {
                shootingActive = false;

                // Stop forced mode if it was accidentally used; safe to call even if not forced
                autoShoot.stopForcedCycle();
                lifter.setDown();

                // ********** CRITICAL: when fireAtStart was used we may never have started follower.
                // Make sure we start the current path (usually pathIndex == 0) if follower not started.
                if (!followerStarted && pathIndex < paths.paths.length) {
                    follower.followPath(paths.paths[pathIndex], true);
                    followerStarted = true;
                    pathTimer.resetTimer();
                    pathState = 1;
                } else {
                    // Advance to next path and start following (unless we're at the end)
                    pathIndex++;
                    if (pathIndex < paths.paths.length) {
                        pathTimer.resetTimer();
                        follower.followPath(paths.paths[pathIndex], true);
                        followerStarted = true;
                    } else {
                        pathState = -1; // finished
                    }
                }
            }
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        handleShooting();

        //telemetry getter
        // --- Update spindexer & autoShoot ---
        hood.update();
        Hood.TelemetryPacket H = hood.getTelemetry();
        lifter.TelemetryPacket l = lifter.getTelemetry();

        telemetry.addData("pathIndex", pathIndex);
        telemetry.addData("pathState", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addLine();

// --- TELEMETRY ---
// SHOOTER
        ShooterClass.ShooterTelemetry s = shooter.getTelemetry();
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Mode", s.mode);
        telemetry.addData("Target RPM", "%.2f", s.targetRPM);
        telemetry.addData("Current RPM", "%.1f", s.currentRPM);
        telemetry.addData("Error RPM", "%.1f", s.errorRPM);
        telemetry.addData("volt", s.motorPower);
        telemetry.addData("pose", s.pose);
        telemetry.addData("spin up time", s.spinupTimeSec);


// SPINDEXER
        telemetry.addLine("=== SPINDEXER ===");
        spin.sampleSensorsNow();
        spin.update();             // 2️⃣ process logic
        Spindexer.TelemetryPacket spina = spin.getTelemetry(); // 3️⃣ snapshot
        telemetry.addData("Mode", spina.mode);
        telemetry.addData("Voltage", "%.3f", spina.voltage);
        telemetry.addData("Angle", "%.1f°", spina.currentAngle);
        telemetry.addData("Target", "%.1f°", spina.targetAngle);
        telemetry.addData("Error", "%.1f°", spina.angleError);
        telemetry.addData("Power", "%.2f", spina.appliedPower);

// TURRET
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Mode", turret.mode);
        telemetry.addData("Power", turret.turretPower);
        telemetry.addData("pose", turret.getPose());
        telemetry.addData("id", turret.getID());
        telemetry.addData("pipeline", turret.limelight.getStatus().getPipelineIndex());

// SILOS
        telemetry.addLine("=== SILOS ===");
        Spindexer.BallColor[] silos = spina.siloColors;
        for (int i = 0; i < silos.length; i++) {
            String label = "Silo " + (i+1);
            // Highlight the current silo
            telemetry.addData(label, silos[i]);
        }
//lift

        telemetry.addLine("=== LIFT ===");
        telemetry.addData("pose", l.position);
        telemetry.addData("volt", l.voltage);
        telemetry.addData("err", l.error);
        telemetry.addData("mode", l.mode);
//hood

        telemetry.addLine("=== HOOD ===");
        telemetry.addData("Mode", H.mode);
        telemetry.addData("Voltage", "%.3f", H.voltage);
        telemetry.addData("Angle", "%.1f°", H.currentAngle);
        telemetry.addData("Target", "%.1f°", H.targetAngle);
        telemetry.addData("Error", "%.1f°", H.angleError);
        telemetry.addData("Power", "%.2f", H.appliedPower);

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
        parem = new Parameters();

        parem.autonomous = Parameters.AUTONOMOUS.YES;

        autoShoot = new ShooterExecutionClass(spin, shooter, hardwareMap, lifter);
        patternMgr = new ShotPatternManager();
        autoShoot.setPatternManager(patternMgr);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        shooter.update(false, false, true);

        turret.limelight.pipelineSwitch(2);
        int tagId = turret.getID();
        ShotPatternManager.ShotPattern p = patternFromTag(tagId);
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        shooter.update(false, false, true);
        opmodeTimer.resetTimer();
        pathIndex = 0;
        setPathState(0);

        if (Parameters.allianceColor == Parameters.Color.BLUE) {
            alliance = Turret.turretAlliance.BLUE;
            turret.limelight.pipelineSwitch(3);
        } else {
            alliance = Turret.turretAlliance.RED;
            turret.limelight.pipelineSwitch(4);
        }

        // Optionally shoot before any path runs:
        if (fireAtStart) {
            shootingActive = true;
            waitTimer.reset();
            // Use pattern-aware startCycle (so ShotPatternManager is used)
            turret.autoMode();
            turret.mode = Turret.turretMode.AUTO;
            patternMgr.clear();
            patternMgr.addPattern(patternFromTag(turret.getID()).sequence);
            autoShoot.startCycle();
            follower.breakFollowing();   // ensure follower not running
            followerStarted = false;    // we'll start follower after initial shooting
        } else {
            // start following path 0 immediately
            follower.followPath(paths.paths[0]);
            followerStarted = true;
        }
    }

    private void startPatternedShooting() {
        ShotPatternManager.ShotPattern p = patternFromTag(turret.getID());
        if (p != null) {
            patternMgr.clear();
            patternMgr.addPattern(p.sequence);
        }
        autoShoot.startCycle();
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}

