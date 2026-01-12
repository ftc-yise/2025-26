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
 * Spindexer (ownership fix)
 *
 * Key change: sensor ownership is now computed as:
 *   sensorAngle = normalize(currentSpindexerAngle + SENSOR_OFFSETS[i])
 *   siloIndex   = angleToSilo(sensorAngle)
 * This guarantees correct sensor -> angle -> silo mapping.
 *
 * Rest of class and API preserved.
 */
public class Spindexer {

    // ─────────────────────────────────────────────────────────────────────
    // MODES
    // ─────────────────────────────────────────────────────────────────────
    public BallColor[] siloColors = new BallColor[3];
    private int[] lastSeenSiloBySensor = { -1, -1, -1 };

    // Unified PID gains (tune these at runtime)
    private static final double kP = 0.0045;
    private static final double MAX_POWER = 0.5;
    private static final double MIN_POWER = 0.04;
    private static final double DEADBAND = 1;
    private static final double SLOW_ZONE = 13.0;


    public static double silo1 = 2;
    public static double silo2 = 244;
    public static double silo3 = 123;

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
            silo1,   // SILO_1
            silo2,   // SILO_2
            silo3    // SILO_3
    };

    private static final double[] SENSOR_OFFSETS = {
            0.0,    // middle
            120.0,  // backLeft
            240.0   // backRight
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
    private final double ANGLE_TOLERANCE = 4.0;

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

    public void goToSilo1() { targetAngleDeg = silo1; mode = Mode.SILO_1; }
    public void goToSilo2() { targetAngleDeg = silo2; mode = Mode.SILO_2; }
    public void goToSilo3() { targetAngleDeg = silo3; mode = Mode.SILO_3; }

    public void setManual(double power) {
        manualPower = -power;
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
                setTarget(silo2);
                if (atTarget()) siloStep++;
                break;

            case 1:
                setTarget(silo3);
                if (atTarget()) siloStep++;
                break;

            case 2:
                setTarget(silo1);
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

        double output = 0.0;
        if (mode == Mode.MANUAL) {
            output = manualPower;
        } else if (mode != Mode.NEUTRAL) {
            output = computeSpindexerPower(current, targetAngleDeg);
        }

        spindexer.setPower(output);

        // --- populate telemetry packet (unchanged structure) ---
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = angleError;
        t.manualPower = manualPower;

        t.pidP = kP * angleError;
        t.pidI = 0.0;
        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }

        writeCSV(current, targetAngleDeg, angleError);

        // --- SENSOR READ / OWNERSHIP PHASE ---
        if (sensorUpdatesEnabled) {
            // Map every sensor reading to the silo that currently sits under that sensor.
            mapSensorsToSilos(current);
        }

        // When neutral, clear last seen ownerships (keeps indexing clean)
        if (mode == Mode.NEUTRAL) {
            Arrays.fill(lastSeenSiloBySensor, -1);
        }
    }

    // --- UPDATE CONTROL ---
    private double computeSpindexerPower(double currentAngle, double targetAngle) {

        double error = smallestAngleError(targetAngle, currentAngle);
        double absError = Math.abs(error);

        // Snap to zero when close enough
        if (absError < DEADBAND) {
            return 0.0;
        }

        // Proportional control
        double power = kP * error;

        // Slow down near target (hard damping, not PID)
        if (absError < SLOW_ZONE) {
            power *= absError / SLOW_ZONE;
        }

        // Minimum power to overcome stiction
        if (Math.abs(power) < MIN_POWER) {
            power = Math.signum(power) * MIN_POWER;
        }

        // Final clamp
        return Range.clip(power, -MAX_POWER, MAX_POWER);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Map sensors -> angle -> silo index (ownership fixer)
    // ─────────────────────────────────────────────────────────────────────
    private void mapSensorsToSilos(double currentAngle) {
        ColorSensor[] sensors = { middle, backLeft, backRight };

        for (int sensorIdx = 0; sensorIdx < sensors.length; sensorIdx++) {
            ColorSensor s = sensors[sensorIdx];
            // compute the world angle that this sensor is "looking at"
            double sensorAngle = normalize(currentAngle + SENSOR_OFFSETS[sensorIdx]);

            // find nearest silo index for that angle
            int siloIndex = angleToSilo(sensorAngle);

            if (siloIndex != -1) {
                // update the silo slot with this sensor's reading
                silos[siloIndex] = detectBall(s, sensorIdx);
                lastSeenSiloBySensor[sensorIdx] = siloIndex;
            } else {
                // sensor not pointing at any known silo right now
                lastSeenSiloBySensor[sensorIdx] = -1;
            }
        }
    }

    // Call ONLY when aligned at a read/fire position (keeps compatibility with existing code)
    public void sampleSensorsNow() {
        double voltage = encoder.getVoltage();
        double current = normalize((voltage / MAX_VOLTAGE) * 360.0);

        // map sensors -> silos
        mapSensorsToSilos(current);

        // copy into telemetry packet so callers reading getTelemetry() see the update immediately
        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
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
    private void writeCSV(double angle, double target, double error) {
        if (csv == null) return;
        try {
            csv.write(System.currentTimeMillis() + "," +
                    df.format(angle) + "," +
                    df.format(target) + "," +
                    df.format(error) + "," +
                    df.format(t.pidP) + ",0,0,");
            csv.flush();
        } catch (IOException ignored) {}
    }

    public TelemetryPacket getTelemetry() { return t; }

    public void stop() {
        spindexer.setPower(0);
        mode = Mode.NEUTRAL;
    }

    // Return a BallColor using sensor thresholds (you already tuned different thresholds per sensor)
    // Return a BallColor using sensor thresholds (sensorIndex: 0=middle, 1=backLeft, 2=backRight)
    private BallColor detectBall(ColorSensor s, int sensorIndex) {
        int g = s.green();
        int b = s.blue();

        if (sensorIndex == 0) { // middle sensor
            if (b > 340) return BallColor.PURPLE;
            else if (g > 315) return BallColor.GREEN;
            return BallColor.NONE;
        } else if (sensorIndex == 1) { // backLeft
            if (b > 225) return BallColor.PURPLE;
            else if (g > 175) return BallColor.GREEN;
            return BallColor.NONE;
        } else { // sensorIndex == 2 -> backRight
            if (b > 175) return BallColor.PURPLE;
            if (g > 250) return BallColor.GREEN;
            return BallColor.NONE;
        }
    }


    public void clearSilo(int index) {
        if (index >= 0 && index < silos.length) {
            silos[index] = BallColor.NONE;
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
}
