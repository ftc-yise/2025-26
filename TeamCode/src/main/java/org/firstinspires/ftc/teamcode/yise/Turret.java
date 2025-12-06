package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    LLResult result = null;
    public static double myOffset = 0.2;
    public static double mySlope = 0.2;
    private double turretPower = 0.0;
    private double myTy = 0.0;
    public DcMotor turret;
    public Limelight3A limelight;
    public Telemetry telemetry;
    public enum turretAlliance{
        RED,
        BLUE,
    }
    public enum turretDirection {
        LEFT,
        RIGHT,
        STOP,
    }
    public enum turretMode{
        AUTO,
        MANUAL,
    }
    public turretMode mode;

    public Turret (HardwareMap hardwareMap, turretAlliance alliance, Telemetry telem){
        telemetry = telem;

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        if (alliance == turretAlliance.RED) {
            limelight.pipelineSwitch(4); // Switch to pipeline number 4 which is ID:24
        }else if (alliance== turretAlliance.BLUE) {
            limelight.pipelineSwitch(0); // Switch to pipeline number 0 which is ID:??
        }
        mode = turretMode.MANUAL;
    }

    public void changeMode() {
        if (mode == turretMode.AUTO) {
            mode = turretMode.MANUAL;
        } else if (mode == turretMode.MANUAL) {
            mode = turretMode.AUTO;
        }
    }
    public void manualMode(turretDirection direction) {
        mode = turretMode.MANUAL;

        switch (direction) {
            case LEFT:
                turret.setPower(-0.7);
                break;
            case RIGHT:
                turret.setPower(0.7);
                break;
            case STOP:
                turret.setPower(0);
                break;
        }
    }
    public void autoMode(){
        mode = turretMode.AUTO;

        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            myTy = result.getTy();
            turretPower = getTurretPower(myTy, myOffset, mySlope);
            //telemetry.addData("Ty=", myTy);
        } else {
            turretPower = 0;
        }
        //telemetry.addData("Power=", turretPower);
        //telemetry.update();
        turret.setPower(turretPower);
    }
    public void stop(){
        turret.setPower(0);
    }
    private double getTurretPower (double ty, double myOffset, double mySlope){
        if (ty < 0) {
            return -.2*(ty * mySlope + myOffset);
        } else if (ty > 0){
            return -.2*(ty * mySlope - myOffset);
        }
        else {
            return 0;
        }
    }
}