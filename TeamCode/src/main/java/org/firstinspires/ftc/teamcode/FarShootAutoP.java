package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "CloseShootAuto_Math", group = "auto")
public class FarShootAutoP extends LinearOpMode {

    // ─────────────────────────────
    // SUBSYSTEMS
    // ─────────────────────────────
    private ShooterClass shooter;
    private ShooterExecutionClass shooterExec;
    private Spindexer spindexer;
    private Hood hood;
    private lifter lifter;
    private Turret turret;
    private DcMotor intake;

    private Follower follower;

    // ─────────────────────────────
    // POSES
    // ─────────────────────────────
    private final Pose p0 = new Pose(18.0, 132.0, Math.toRadians(315));
    private final Pose p1 = new Pose(60.0, 108.0, Math.toRadians(315));
    private final Pose p2 = new Pose(36.0, 84.0, Math.toRadians(180));

    private PathChain pathP0_P1;
    private PathChain pathP1_P2;

    // ─────────────────────────────
    // AUTO STATE MACHINE
    // ─────────────────────────────
    private enum AutoState {
        START,
        DRIVE_P0_P1,
        TURRET_ALIGN,
        SPINUP,
        SHOOT,
        DRIVE_P1_P2,
        DONE
    }

    private AutoState state = AutoState.START;
    private final ElapsedTime stateTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ───────── INIT SUBSYSTEMS ─────────
        shooter = new ShooterClass(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        hood = new Hood(hardwareMap);
        lifter = new lifter(hardwareMap);
        shooterExec = new ShooterExecutionClass(spindexer, shooter, lifter);
        turret = new Turret(hardwareMap, Turret.turretAlliance.BLUE, telemetry);

        intake = hardwareMap.get(DcMotor.class, "intake");

        // ───────── INIT FOLLOWER + PATHS ─────────
        follower.setStartingPose(p0);

        pathP0_P1 = follower.pathBuilder()
                .addPath(new BezierLine(p0, p1))
                .build();

        pathP1_P2 = follower.pathBuilder()
                .addPath(new BezierLine(p1, p2))
                .build();

        // ───────── INIT SUBSYSTEM SETTINGS ─────────
        hood.setTarget(40);       // fixed close shot
        lifter.setDown();
        spindexer.goToSilo1();

        waitForStart();
        stateTimer.reset();

        // ───────── MAIN LOOP ─────────
        while (opModeIsActive()) {

            // ───── UPDATE ALL SUBSYSTEMS ─────
            follower.update();
            shooter.update(false, false, true);
            shooterExec.update();
            spindexer.update();
            hood.update();
            lifter.update();
            turret.autoMode();

            // ───── STATE MACHINE ─────
            switch (state) {

                case START:
                    follower.followPath(pathP0_P1);
                    state = AutoState.DRIVE_P0_P1;
                    break;

                case DRIVE_P0_P1:
                    if (!follower.isBusy()) {
                        turret.runto(60); // encoder position for auto-align
                        stateTimer.reset();
                        state = AutoState.TURRET_ALIGN;
                    }
                    break;

                case TURRET_ALIGN:
                    if (stateTimer.seconds() > 0.5) {
                        stateTimer.reset();
                        state = AutoState.SPINUP;
                    }
                    break;

                case SPINUP:
                    if (shooter.getTelemetry().readyLoose || stateTimer.seconds() > 1.5) {
                        shooterExec.startCycle();
                        state = AutoState.SHOOT;
                    }
                    break;

                case SHOOT:
                    if (!shooterExec.isBusy()) {
                        intake.setPower(1.0);
                        follower.followPath(pathP1_P2);
                        state = AutoState.DRIVE_P1_P2;
                    }
                    break;

                case DRIVE_P1_P2:
                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    shooter.stop();
                    hood.stop();
                    return;
            }

            // ───── TELEMETRY ─────
            telemetry.addData("AutoState", state);
            telemetry.addData("Shooter RPM", shooter.getTelemetry().currentRPM);
            telemetry.addData("Shots Fired", shooterExec.getShotLog().length);
            telemetry.update();
        }
    }
}
