package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterClass {

    // Shooter modes
    public enum ShooterMode { STOP, IDLE, FULL, LOW }
    private ShooterMode shooterMode = ShooterMode.STOP;

    // Hardware
    private DcMotorEx shooter = null;
    private DcMotor shooterLegacy = null;

    // Power levels
    private final double POWER_STOP = 0.0;
    private final double POWER_IDLE = 0.05;
    private final double POWER_FULL = 0.4;
    private final double POWER_LOW  = 0.10;

    // Telemetry storage
    public static class ShooterTelemetry {
        public ShooterMode mode;
        public double appliedPower;
        public double velocity;
    }
    private ShooterTelemetry t = new ShooterTelemetry();

    public ShooterClass(HardwareMap hardwareMap) {
        try {
            shooter = hardwareMap.get(DcMotorEx.class, "shoot");
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            shooterLegacy = hardwareMap.get(DcMotor.class, "shoot");
            shooterLegacy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterLegacy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        }
    }

    // --- MAIN UPDATE ---
    public void update(boolean a, boolean b, boolean x, boolean y) {
        // A = STOP
        // B = IDLE
        // X = LOW (60%)
        // Y = FULL (85%)

        if (y) shooterMode = ShooterMode.FULL;
        else if (x) shooterMode = ShooterMode.LOW;
        else if (!x && !y && !b && !a) shooterMode = ShooterMode.IDLE;
        else if (a) shooterMode = ShooterMode.STOP;

        double p = 0;

        switch (shooterMode) {
            case STOP: p = POWER_STOP; break;
            case IDLE: p = POWER_IDLE; break;
            case FULL: p = POWER_FULL; break;
            case LOW:  p = POWER_LOW;  break;
        }

        setPower(p);

        // Save telemetry
        t.mode = shooterMode;
        t.appliedPower = p;
        t.velocity = getVelocity();
    }

    private void setPower(double p) {
        if (shooter != null) shooter.setPower(p);
        else if (shooterLegacy != null) shooterLegacy.setPower(p);
    }

    private double getVelocity() {
        if (shooter != null) {
            try { return shooter.getVelocity(); }
            catch (Exception e) { return 0; }
        }
        return 0;
    }

    // Public getter for main OpMode
    public ShooterTelemetry getTelemetry() { return t; }

    public void stop() {
        setPower(0);
        shooterMode = ShooterMode.STOP;
    }
}
