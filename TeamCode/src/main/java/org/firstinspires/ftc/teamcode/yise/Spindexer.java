package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;

public class Spindexer {

    // ─────────────────────────────────────────────────────────────────────
    // MODES
    // ─────────────────────────────────────────────────────────────────────
    public enum Mode {
        NEUTRAL,
        SILO_1,
        SILO_2,
        SILO_3,
        MANUAL,
        TARGET,
        SEQUENCE,        // NEW: automatic silo sequence
    }

    public Mode mode = Mode.NEUTRAL;

    // Hardware
    private CRServo spindexer;
    private AnalogInput encoder;

    // Angle variables
    private double targetAngleDeg = 0;
    private double manualPower = 0;

    // Constants
    private final double MAX_VOLTAGE = 3.3;

    // PID constants
    private final double kP = 0.022;
    private final double kI = 0.00035;
    private final double kD = 0.001;

    // PID state
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = System.nanoTime();

    // Drift autocorrect timer
    private long lastCorrectionTime = 0;
    private final long CORRECTION_INTERVAL_MS = 2250; // every ~2.2 sec
    private final double DRIFT_THRESHOLD = 7;         // deg

    // Rate limiting (smooth accel)
    private double lastPower = 0;
    private final double MAX_DELTA = 0.04; // accel limit per update

    // Sequencing state
    private int siloStep = 0;
    private boolean sequenceActive = false;

    // Angle tolerance
    private final double ANGLE_TOLERANCE = 5.0;

    // ─────────────────────────────────────────────────────────────────────
    // TELEMETRY STRUCT
    // ─────────────────────────────────────────────────────────────────────
    public static class TelemetryPacket {
        public Mode mode;
        public double voltage;
        public double currentAngle;
        public double targetAngle;
        public double angleError;
        public double appliedPower;
        public double manualPower;
        public double pidP;
        public double pidI;
        public double pidD;
    }

    private TelemetryPacket t = new TelemetryPacket();

    // CSV Logging
    private FileWriter csv;
    private DecimalFormat df = new DecimalFormat("0.00");

    // ─────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────
    public Spindexer(HardwareMap hardwaremap) {
        spindexer = hardwaremap.get(CRServo.class, "spin");
        encoder = hardwaremap.get(AnalogInput.class, "spinInput");
    }

    // ─────────────────────────────────────────────────────────────────────
    // MODE COMMANDS
    // ─────────────────────────────────────────────────────────────────────
    public void setNeutral() {
        mode = Mode.NEUTRAL;
        sequenceActive = false;
    }

    public void goToSilo1() { targetAngleDeg = 120; mode = Mode.SILO_1; }
    public void goToSilo2() { targetAngleDeg = 240; mode = Mode.SILO_2; }
    public void goToSilo3() { targetAngleDeg = 0;   mode = Mode.SILO_3; }

    public void setManual(double power) {
        manualPower = power;
        mode = Mode.MANUAL;
    }

    public void setTarget(double angleDeg) {
        targetAngleDeg = normalize(angleDeg);
        mode = Mode.TARGET;
    }

    // ─────────────────────────────────────────────────────────────────────
    // AUTOMATIC SILO SEQUENCE
    // ─────────────────────────────────────────────────────────────────────
    public void startSequence() {
        siloStep = 0;
        sequenceActive = true;
        mode = Mode.SEQUENCE;
    }

    private void runSequence() {
        if (!sequenceActive) return;

        switch (siloStep) {
            case 0:
                setTarget(120);
                if (atTarget()) siloStep++;
                break;

            case 1:
                setTarget(240);
                if (atTarget()) siloStep++;
                break;

            case 2:
                setTarget(0);
                if (atTarget()) siloStep++;
                break;

            case 3:
                sequenceActive = false;
                setNeutral();
                break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // UPDATE LOOP — CALL EVERY LOOP
    // ─────────────────────────────────────────────────────────────────────
    public void update() {

        // Read angle
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // Auto-sequence logic
        if (mode == Mode.SEQUENCE) runSequence();

        double angleError = smallestAngleError(targetAngleDeg, current);

        double power;

        if (mode == Mode.MANUAL) {
            power = manualPower;

        } else if (mode == Mode.NEUTRAL) {
            power = 0;

        } else {
            power = pidControl(angleError);
        }

        // Drift auto-correction
        long now = System.currentTimeMillis();
        if (now - lastCorrectionTime > CORRECTION_INTERVAL_MS &&
                Math.abs(angleError) < ANGLE_TOLERANCE &&
                Math.abs(angleError) > DRIFT_THRESHOLD)
        {
            targetAngleDeg = current;
            lastCorrectionTime = now;
        }

        // Smooth acceleration
        power = rateLimit(power);

        // Apply power
        spindexer.setPower(power);

        // Telemetry
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = angleError;
        t.appliedPower = power;
        t.manualPower = manualPower;

        // Log to CSV
        writeCSV(current, targetAngleDeg, angleError, power);
    }

    // ─────────────────────────────────────────────────────────────────────
    // PID CONTROL
    // ─────────────────────────────────────────────────────────────────────
    private double pidControl(double error) {
        double dt = (System.nanoTime() - lastTime) / 1e9;
        lastTime = System.nanoTime();

        // Integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -50, 50);

        // Derivative
        double derivative = (error - lastError) / dt;
        derivative = derivative * 0.7; // low-pass filter smoothing

        lastError = error;

        // PID output
        double p = kP * error;
        double i = kI * integral;
        double d = kD * derivative;

        // Save telemetry PID terms
        t.pidP = p;
        t.pidI = i;
        t.pidD = d;

        return p + i + d;
    }

    // ─────────────────────────────────────────────────────────────────────
    // RATE LIMITER
    // ─────────────────────────────────────────────────────────────────────
    private double rateLimit(double p) {
        double diff = p - lastPower;

        if (Math.abs(diff) > MAX_DELTA) {
            p = lastPower + Math.signum(diff) * MAX_DELTA;
        }

        lastPower = p;
        return clamp(p, -1, 1);
    }

    // ─────────────────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────────────────
    private double normalize(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private boolean atTarget() {
        return Math.abs(lastError) < ANGLE_TOLERANCE;
    }

    private double smallestAngleError(double target, double current) {
        double diff = target - current;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        return diff;
    }

    private double clamp(double p, double min, double max) {
        return Math.max(min, Math.min(max, p));
    }

    private double clamp(double p) {
        return clamp(p, -1, 1);
    }

    // CSV logger
    private void writeCSV(double angle, double target, double error, double power) {
        if (csv == null) return;
        try {
            csv.write(System.currentTimeMillis() + "," +
                    df.format(angle) + "," +
                    df.format(target) + "," +
                    df.format(error) + "," +
                    df.format(t.pidP) + "," +
                    df.format(t.pidI) + "," +
                    df.format(t.pidD) + "," +
                    df.format(power) + "\n");
            csv.flush();
        } catch (IOException ignored) {}
    }

    // Public telemetry getter
    public TelemetryPacket getTelemetry() { return t; }

    public void stop() {
        spindexer.setPower(0);
        mode = Mode.NEUTRAL;
    }
}
