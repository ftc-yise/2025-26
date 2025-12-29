package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Spindexer
 *
 * Fixed / cleaned control loop:
 * - single set of PID gains
 * - velocity estimate for D term
 * - simple velocity feedforward for braking (kV)
 * - sensor gating only skips sensor reads (doesn't return early)
 *
 * Architecture / public API unchanged.
 */
public class Spindexer {

    // ─────────────────────────────────────────────────────────────────────
    // MODES
    // ─────────────────────────────────────────────────────────────────────
    public BallColor[] siloColors = new BallColor[3];
    private int[] lastSeenSiloBySensor = { -1, -1, -1 };

    // Unified PID gains (tune these at runtime)
    public double kP = 0.015;
    public double kI = 0.0;        // kept but unused (no I implemented)
    public double kD = 0.0015;

    public enum Mode {
        NEUTRAL,
        SILO_1,
        SILO_2,
        SILO_3,
        MANUAL,
        TARGET,
        SEQUENCE
    }
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    private final BallColor[] silos = {
            BallColor.NONE,
            BallColor.NONE,
            BallColor.NONE
    };

    private static final double[] SILO_ANGLES = {
            17.5,   // SILO_1
            139.0,  // SILO_2
            257.0   // SILO_3
    };

    private static final double[] SENSOR_OFFSETS = {
            0.0,    // middle
            120.0,  // backLeft
            240.0   // backRight
    };

    // Read windows for each sensor (custom positions)
    private static final double[][] SENSOR_READ_ANGLES = {
            { 10, 25 },    // middle sensor read window
            { 130, 150 },  // backLeft read window
            { 250, 270 }   // backRight read window
    };

    public Mode mode = Mode.NEUTRAL;

    // Hardware
    private CRServo spindexer;
    private AnalogInput encoder;
    private ColorSensor middle = null;
    private ColorSensor backLeft = null;
    private ColorSensor backRight = null;

    // Angle variables
    public double targetAngleDeg = 0;
    private double manualPower = 0;

    // Constants
    private final double MAX_VOLTAGE = 3.3;

    // Drift autocorrect timer
    private long lastCorrectionTime = 0;
    private final long CORRECTION_INTERVAL_MS = 2250;
    private final double DRIFT_THRESHOLD = 7;

    // reading gate
    private boolean sensorUpdatesEnabled = true;

    // Velocity estimate helpers
    private double lastAngle = 0.0;            // last measured angle (deg)
    private long lastTimeMs = System.currentTimeMillis();

    // Feedforward and output limiting (tune these)
    private double kV = 0.0025;                // velocity -> power feedforward
    private double maxOutput = 0.5;           // clamp output to safe range

    // Rate limiting (keeps power changes smooth)
    private double lastPower = 0;
    private double MAX_DELTA = 0.04;

    // Sequencing state
    private int siloStep = 0;
    private boolean sequenceActive = false;

    // Angle tolerance for various checks
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
        public BallColor[] siloColors = new BallColor[3];
        // additional telemetry helpers could be added here
    }

    private TelemetryPacket t = new TelemetryPacket();

    // CSV Logging
    private FileWriter csv;
    private DecimalFormat df = new DecimalFormat("0.00");

    // Constructor
    public Spindexer(HardwareMap hardwaremap) {
        spindexer = hardwaremap.get(CRServo.class, "spin");
        encoder = hardwaremap.get(AnalogInput.class, "spinInput");

        middle = hardwaremap.get(ColorSensor.class, "middlecolorsensor");
        backLeft = hardwaremap.get(ColorSensor.class, "BLcolorsensor");
        backRight = hardwaremap.get(ColorSensor.class, "BRcolorsensor");

        // init lastAngle so first velocity estimate is small
        double v = encoder.getVoltage();
        lastAngle = normalize((v / MAX_VOLTAGE) * 360.0);
        lastTimeMs = System.currentTimeMillis();
    }

    // ─────────────────────────────────────────────────────────────────────
    // MODE COMMANDS
    // ─────────────────────────────────────────────────────────────────────
    public void setNeutral() {
        mode = Mode.NEUTRAL;
        sequenceActive = false;
    }

    public void goToSilo1() { targetAngleDeg = 17.5; mode = Mode.SILO_1; }
    public void goToSilo2() { targetAngleDeg = 139; mode = Mode.SILO_2; }
    public void goToSilo3() { targetAngleDeg = 257; mode = Mode.SILO_3; }

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

        // --- read encoder / current angle ---
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // run sequence if needed
        if (mode == Mode.SEQUENCE) runSequence();

        // controller error (target minus current)
        double angleError = smallestAngleError(targetAngleDeg, current);

        // Choose base control mode (manual, neutral, or closed-loop)
        double rawOutput = 0.0;
        if (mode == Mode.MANUAL) {
            // driver controlled power
            rawOutput = manualPower;
        } else if (mode == Mode.NEUTRAL) {
            // no motion
            rawOutput = 0.0;
        } else {
            // TARGET or SEQUENCE: use PD + feedforward
            // --- velocity estimate (deg/sec) ---
            long nowMs = System.currentTimeMillis();
            double dt = (nowMs - lastTimeMs) / 1000.0;
            if (dt <= 0.0) dt = 1e-3;

            // delta must be computed with wrap-safe function
            double deltaAngle = smallestAngleError(current, lastAngle); // current - lastAngle
            double velocity = deltaAngle / dt; // deg/sec

            // PD terms
            double pTerm = kP * angleError;
            double dTerm = -kD * velocity; // negative sign: if velocity towards setpoint, reduce power

            // simple braking/feedforward term (maps desired slow velocity to power)
            double targetVel = Range.clip(angleError * 0.04, -90.0, 90.0); // deg/sec desired
            double ff = targetVel * kV;

            // combine
            double pidOut = pTerm + dTerm;
            double combined = pidOut + ff;

            // soft-landing scaling when close to target
            double scale = Range.clip(Math.abs(angleError) / 30.0, 0.25, 1.0);
            rawOutput = combined * scale;

            // update velocity / time history
            lastAngle = current;
            lastTimeMs = nowMs;
        }

        // apply rate limiting (smooth transitions) and final clamp
        double limited = rateLimit(rawOutput);
        limited = Range.clip(limited, -maxOutput, maxOutput);

        // set the motor once here (single source of motor commands)
        spindexer.setPower(limited);

        // --- populate telemetry packet (unchanged structure) ---
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = angleError;
        t.appliedPower = limited;
        t.manualPower = manualPower;

        t.pidP = kP * angleError;
        t.pidI = 0.0;
        t.pidD = kD;
        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }

        writeCSV(current, targetAngleDeg, angleError, limited);

        // --- SENSOR READ PHASE (only this block is skipped when disabled) ---
        if (sensorUpdatesEnabled) {
            ColorSensor[] sensors = { middle, backLeft, backRight };

            for (int i = 0; i < 3; i++) {
                double effectiveAngle = normalize(current + SENSOR_OFFSETS[i]);

                if (inWindow(
                        effectiveAngle,
                        SENSOR_READ_ANGLES[i][0],
                        SENSOR_READ_ANGLES[i][1])) {

                    // update silo assigned to this sensor's window
                    silos[i] = detectBall(sensors[i], i);

                    // record last seen silo mapping (useful later)
                    lastSeenSiloBySensor[i] = i;
                }
            }
        }

        // When neutral, clear last seen ownerships (keeps indexing clean)
        if (mode == Mode.NEUTRAL) {
            Arrays.fill(lastSeenSiloBySensor, -1);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // P-only control (legacy support) - retained but not used for TARGET mode
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

    // Return a BallColor using sensor thresholds (you already tuned different thresholds per sensor)
    private BallColor detectBall(ColorSensor s, int i) {
        int g = s.green();
        int b = s.blue();
        if (i == 1) {
            if (g > 200 && b > 700) return BallColor.PURPLE;
            if (g > 200) return BallColor.GREEN;
            return BallColor.NONE;
        } else if (i == 2) {
            if (b > 2000) return BallColor.PURPLE;
            else if (g > 1750) return BallColor.GREEN;
            return BallColor.NONE;
        } else {
            if (b > 2002) return BallColor.PURPLE;
            else if (g > 2000) return BallColor.GREEN;
            return BallColor.NONE;
        }
    }

    private int angleToSilo(double angle) {
        for (int i = 0; i < 3; i++) {
            double error = smallestAngleError(SILO_ANGLES[i], angle);
            if (Math.abs(error) < ANGLE_TOLERANCE) return i;
        }
        return -1;
    }

    public void enableSensorUpdates() {
        sensorUpdatesEnabled = true;
    }

    public void disableSensorUpdates() {
        sensorUpdatesEnabled = false;
    }

    private boolean inWindow(double angle, double min, double max) {
        angle = normalize(angle);
        min = normalize(min);
        max = normalize(max);

        if (min < max) return angle >= min && angle <= max;
        return angle >= min || angle <= max;
    }
}
