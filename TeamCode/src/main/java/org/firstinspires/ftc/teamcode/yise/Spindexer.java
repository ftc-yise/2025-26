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
        SEQUENCE
    }

    public enum Silo1 {
        GREEN,
        PURPLE,
        NONE
    }
    public enum Silo2 {
        GREEN,
        PURPLE,
        NONE
    }
    public enum Silo3 {
        GREEN,
        PURPLE,
        NONE
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

    // P-only control
    private final double kP = 0.0058;

    // Drift autocorrect timer
    private long lastCorrectionTime = 0;
    private final long CORRECTION_INTERVAL_MS = 2250;
    private final double DRIFT_THRESHOLD = 7;

    // Rate limiting
    private double lastPower = 0;
    private final double MAX_DELTA = 0.04;

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
        public double pidI;   // always 0 for P-only
        public double pidD;   // always 0 for P-only
    }

    private TelemetryPacket t = new TelemetryPacket();

    // CSV Logging
    private FileWriter csv;
    private DecimalFormat df = new DecimalFormat("0.00");

    // Constructor
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

    public void goToSilo1() { targetAngleDeg = 19; mode = Mode.SILO_1; }
    public void goToSilo2() { targetAngleDeg = 140; mode = Mode.SILO_2; }
    public void goToSilo3() { targetAngleDeg = 259; mode = Mode.SILO_3; }

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

        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        if (mode == Mode.SEQUENCE) runSequence();

        double angleError = smallestAngleError(targetAngleDeg, current);

        double power;

        if (mode == Mode.MANUAL) {
            power = manualPower;

        } else if (mode == Mode.NEUTRAL) {
            power = 0;

        } else {
            power = pControl(angleError);
        }

        long now = System.currentTimeMillis();
        if (now - lastCorrectionTime > CORRECTION_INTERVAL_MS &&
                Math.abs(angleError) < ANGLE_TOLERANCE &&
                Math.abs(angleError) > DRIFT_THRESHOLD)
        {
            targetAngleDeg = current;
            lastCorrectionTime = now;
        }

        power = rateLimit(power);

        spindexer.setPower(power);

        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = angleError;
        t.appliedPower = power;
        t.manualPower = manualPower;

        t.pidP = kP * angleError;
        t.pidI = 0;
        t.pidD = 0;

        writeCSV(current, targetAngleDeg, angleError, power);
    }

    // ─────────────────────────────────────────────────────────────────────
    // P-ONLY CONTROL
    // ─────────────────────────────────────────────────────────────────────
    private double pControl(double error) {
        return kP * error;
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
        return Math.abs(t.angleError) < ANGLE_TOLERANCE;
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

    // CSV logger
    private void writeCSV(double angle, double target, double error, double power) {
        if (csv == null) return;
        try {
            csv.write(System.currentTimeMillis() + "," +
                    df.format(angle) + "," +
                    df.format(target) + "," +
                    df.format(error) + "," +
                    df.format(t.pidP) + ",0,0," +
                    df.format(power) + "\n");
            csv.flush();
        } catch (IOException ignored) {}
    }

    public TelemetryPacket getTelemetry() { return t; }

    public void stop() {
        spindexer.setPower(0);
        mode = Mode.NEUTRAL;
    }
}
