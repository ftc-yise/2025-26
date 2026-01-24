package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;


import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.yise.Hood;
import org.firstinspires.ftc.teamcode.yise.ShooterClass;
import org.firstinspires.ftc.teamcode.yise.ShooterExecutionClass;
import org.firstinspires.ftc.teamcode.yise.Spindexer;
import org.firstinspires.ftc.teamcode.yise.Turret;
import org.firstinspires.ftc.teamcode.yise.lifter;

@Autonomous(name = "CloseShootAuto", group = "auto")
@Disabled
public class CloseShootAutoP extends LinearOpMode {
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    // subsystems
    ShooterClass shooter;
    ShooterExecutionClass autoShoot;
    Spindexer spindexer;
    Hood hood;
    lifter lifter;
    Turret turret;

    DcMotor intake;
    Follower follower;

    // state machine
    enum State {
        DRIVE_TO_P2,
        TURRET_ALIGN,
        SHOOT,
        INTAKE_AND_FINAL_DRIVE,
        DONE
    }

    State state = State.DRIVE_TO_P2;
    ElapsedTime stateTimer = new ElapsedTime();

    // pattern
    ShooterExecutionClass exec;

    @Override
    public void runOpMode() {

        // ---------------- INIT ----------------
        shooter = new ShooterClass(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        hood = new Hood(hardwareMap);
        lifter = new lifter(hardwareMap);
        turret = new Turret(hardwareMap, Turret.turretAlliance.BLUE, telemetry);
        exec = new ShooterExecutionClass(spindexer, shooter, lifter);

        intake = hardwareMap.get(DcMotor.class, "intake");
        follower = Constants.createFollower(hardwareMap);

        // poses (ONLY FIRST THREE + FINAL)
        Pose p0 = new Pose(18.0, 132.0);
        Pose p1 = new Pose(60.0, 108.0);
        Pose p2 = new Pose(36.0, 84.0, Math.toRadians(180));
        //Pose p7 = new Pose(63.729, 8.206, Math.toRadians(180));

        follower.setStartingPose(new Pose(p0.getX(), p0.getY(), Math.toRadians(315)));

        PathChain pathToP2 = follower.pathBuilder()
                .addPath(new BezierCurve(p0, p1))
                .setTangentHeadingInterpolation()
                .build();

        PathChain finalPath = follower.pathBuilder()
                .addPath(new BezierCurve(p1, p2))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        hood.setTarget(40);
        lifter.setDown();
        spindexer.goToSilo1();

        waitForStart();
        stateTimer.reset();

        follower.followPath(pathToP2);

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            // ALWAYS UPDATE
            follower.update();
            shooter.update(false, false, true);
            exec.update();
            spindexer.update();
            hood.update();
            lifter.update();

            turret.mode = Turret.turretMode.AUTO;
            turret.autoMode();

            switch (state) {

                case DRIVE_TO_P2:
                    if (!follower.isBusy()) {
                        turret.runto(60);
                        state = State.TURRET_ALIGN;
                        stateTimer.reset();
                    }
                    break;

                case TURRET_ALIGN:
                    // small settle delay
                    if (stateTimer.seconds() > 0.4) {
                        exec.startCycle();   // SHOOT ALL BALLS
                        state = State.SHOOT;
                        stateTimer.reset();
                    }
                    break;

                case SHOOT:
                    if (!exec.isBusy() && stateTimer.seconds() > 7) {
                        intake.setPower(0.7);
                        follower.followPath(finalPath);
                        state = State.INTAKE_AND_FINAL_DRIVE;
                        stateTimer.reset();
                    }
                    break;

                case INTAKE_AND_FINAL_DRIVE:
                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        state = State.DONE;
                    }
                    break;

                case DONE:
                    shooter.stop();
                    hood.stop();
                    spindexer.setNeutral();
                    return;
            }

            telemetryM.debug("x:" + follower.getPose().getX());
            telemetryM.debug("y:" + follower.getPose().getY());
            telemetryM.debug("heading:" + follower.getPose().getHeading());
            telemetryM.debug("total heading:" + follower.getTotalHeading());
            telemetryM.update(telemetry);

            draw();
        }
    }
}
