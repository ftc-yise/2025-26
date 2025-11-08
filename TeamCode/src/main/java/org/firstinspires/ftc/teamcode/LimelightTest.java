package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="LimelightTest", group="Linear Opmode")
public class LimelightTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //private Limelight3A limelight;
    private double distance_g;
    private DcMotor turret = null;

    private double turretPower = 0.0;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotor.class, "turret");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(4); // Switch to pipeline number 4 which is ID:24*/

        waitForStart();
        runtime.reset();;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double x =botpose.getPosition().x * 39.37;
                double y =botpose.getPosition().y * 39.37;
                distance_g = getDistanceFromPose( x,y );
                telemetry.addData("Distance", distance_g);

                telemetry.addData("Target X", result.getTy());
                telemetry.addData("Target Y", result.getTx());
                telemetry.addData("Target Area", result.getTa());
                telemetry.addData("Botpose", botpose.toString());

                telemetry.addData("MT1 Location", "("+ x +"," + y +")");
                telemetry.update();


            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
            }
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setDirection(DcMotor.Direction.FORWARD);


            if (gamepad1.b) {
                turret.setPower(0.6);
            } else if (gamepad1.x) {
                turret.setPower(-0.6);
            } else {
                turret.setPower(0);
                result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    turretPower = getTurretPower(result.getTx());
                    turret.setPower(turretPower);

                }
            }

        }
    }
    public double getDistanceFromPose(double x, double y) {
       double a =  Math.abs(55 - y);
        double b = Math.abs(-58 - x);
        double c_sqrd = Math.pow(a,2) + Math.pow(b,2);
        double distance = Math.sqrt(c_sqrd);
        return distance;
    }

    public double getTurretPower (double tx){
        double power = tx * 0.0023 + 0.4;
        return power;
    }
}