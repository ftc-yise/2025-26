package org.firstinspires.ftc.teamcode.yise;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;

public class ShooterTelemetryUtil {

    public static void draw(
            Telemetry telemetry,
            ShooterExecutionClass shooterExec,
            Spindexer spindexer
    ) {

        telemetry.addLine("── Shooter Execution ──");

        // Shot log
        int[] log = shooterExec.getShotLog();
        telemetry.addData("Shot Log", Arrays.toString(log));

        // Silo colors
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        telemetry.addData(
                "Silos",
                String.format(
                        "[0]=%s [1]=%s [2]=%s",
                        colors[0],
                        colors[1],
                        colors[2]
                )
        );

        // Pattern state
        telemetry.addData("Busy", shooterExec.isBusy());

        telemetry.addLine("------------------------");
    }
}
