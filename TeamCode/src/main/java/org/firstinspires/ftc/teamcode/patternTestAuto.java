package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.yise.*;

@Autonomous(name = "Pattern Shoot Auto", group = "Auto")
public class patternTestAuto extends LinearOpMode {

    private Spindexer spindexer;
    private ShooterClass shooter;
    private lifter lifter;
    private ShooterExecutionClass shooterExec;

    @Override
    public void runOpMode() {

        // ─────────────────────────────────────────────
        // INIT
        // ─────────────────────────────────────────────
        spindexer = new Spindexer(hardwareMap);
        shooter   = new ShooterClass(hardwareMap);
        lifter    = new lifter(hardwareMap);

        shooterExec = new ShooterExecutionClass(
                spindexer,
                shooter,
                lifter
        );

        // Example Decode 2026 pattern
        // (passed from scouting / vision / prior logic)
        int[] pattern = {0, 1, 2};
        shooterExec.setPattern(pattern);

        telemetry.addLine("Pattern loaded: 0-1-2");
        telemetry.update();

        waitForStart();

        // ─────────────────────────────────────────────
        // AUTO LOOP
        // ─────────────────────────────────────────────
        shooter.update(false,false,true);   // whatever your shooter uses

        while (opModeIsActive()) {

            spindexer.update();
            lifter.update();
            shooterExec.update();

            // Start cycle whenever idle
            if (!shooterExec.isBusy()) {
                shooterExec.startCycle();
            }

            ShooterTelemetryUtil.draw(
                    telemetry,
                    shooterExec,
                    spindexer
            );

            telemetry.update();
        }
    }
}
