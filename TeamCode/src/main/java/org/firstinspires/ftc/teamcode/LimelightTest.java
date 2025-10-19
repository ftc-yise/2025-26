package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="LimelightTest", group="Linear Opmode")
public class LimelightTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private double distance_g;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(4); // Switch to pipeline number 4 which is ID:24

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                distance_g = getDistanceFromTag(result.getTa());
                telemetry.addData("Distance", distance_g);
                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.update();
            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
            }
        }
    }
    public double getDistanceFromTag(double ta) {
        double scale = 23851.19;
        double distance = (scale / ta);
        return distance;
    }
}
