package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {

    public DcMotor turret;
    public enum turretDirection {
        LEFT,
        RIGHT,
    }

    public Turret (HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(4); // Switch to pipeline number 4 which is ID:24
    }

    public void manualTurn(turretDirection direction) {
        switch (direction) {
            case LEFT:
                turret.setPower(-0.7);
                break;
            case RIGHT:
                turret.setPower(0.7);
                break;
        }
    }
    public void stop(){
        turret.setPower(0);
    }
}