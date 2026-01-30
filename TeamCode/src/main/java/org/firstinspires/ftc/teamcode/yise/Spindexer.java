package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.yise.Parameters;

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
    private int[] lastSeenSiloBySensor = {-1, -1, -1};

    // Unified PID gains (tune these at runtime)
    // --- PIDF Gains & helpers (replace old kP/MAX_POWER etc. if present) ---
    // PID-ish tuning (CRServo-friendly)
    private static final double kP = 0.0065
            ;          // proportional gain
    private static final double DEAD_BAND = 2.5;     // degrees
    private static final double HOLD_POWER = 0.015;   // static friction compensation
    private static final double MAX_POWER = 1;
    private static final double kD = 0.00215; // start here

    // --- additional tunable control params (start values, tweak on-robot) ---
    private static double kI = 0.0005;                 // small integrator (usually near 0 for CRServo)
    private static final double VELOCITY_FILTER_ALPHA = 0.18; // 0..1, lower = smoother velocity
    private static final double SLOW_ZONE_DEG = 60.0;  // start braking this far from target
    private static final double MAX_SLOW_POWER = 0.40; // max power allowed while inside SLOW_ZONE
    private static final double MIN_APPROACH_POWER = 0.055; // small nudge to prevent stall
    private static final double INTEGRATOR_MAX = 0.15; // anti-windup clamp
    private static final double STOP_VELOCITY = 12.0;  // deg/s considered "stopped"
    private double integrator = 0.0;


    private double lastError = 0;
    private long lastTimeNs = 0;


    // filters and rate limiting
    private double lastFilteredAngle = 0.0;
    private double lastFilteredVelocity = 0.0;

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

    public static double silo1;
    public static double silo2;
    public static double silo3;

    // keep your existing power limits and deadband (tweakable)
    public static double[] SILO_ANGLES = {
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

    // reading gate
    private boolean sensorUpdatesEnabled = true;

    // Velocity estimate helpers
    private double lastAngle = 0.0;            // last measured angle (deg)
    private long lastTimeMs = System.currentTimeMillis();

    // Rate limiting (keeps power changes smooth)
    private double lastPower = 0;
    private double MAX_DELTA = 0.12;

    // Sequencing state
    private int siloStep = 0;
    private boolean sequenceActive = false;

    // Angle tolerance for various checks
    private final double ANGLE_TOLERANCE = 0.5;
    private final double ANGLE_TOLERANCE_COLOR = 15;

    // --- arrival latch ---
    private boolean atTargetLatched = false;


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
        lastFilteredAngle = lastAngle;     // <--- add this line
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

        // ALWAYS update sensors (telemetry must never starve)
        if (sensorUpdatesEnabled) {
            mapSensorsToSilos(current);
        }

        // run sequence if needed
        if (mode == Mode.SEQUENCE) runSequence();

        double output = 0.0;

        if (mode == Mode.MANUAL) {
            output = manualPower;
            atTargetLatched = false;
        }
        else if (mode != Mode.NEUTRAL) {
            output = computeSpindexerPower(current, targetAngleDeg);
        }
        else {
            output = 0.0;
            atTargetLatched = false;
        }

        output = rateLimit(output);
        spindexer.setPower(output);

        // --- telemetry packet ---
        t.mode = mode;
        t.voltage = voltage;
        t.currentAngle = current;
        t.targetAngle = targetAngleDeg;
        t.angleError = smallestAngleError(targetAngleDeg, current);
        t.appliedPower = output;
        t.manualPower = manualPower;

        for (int i = 0; i < 3; i++) {
            t.siloColors[i] = silos[i];
        }

        writeCSV(current, targetAngleDeg, t.angleError);

        if (mode == Mode.NEUTRAL) {
            Arrays.fill(lastSeenSiloBySensor, -1);
        }
    }

    // --- UPDATE CONTROL ---
    private double computeSpindexerPower(double currentAngle, double targetAngle) {
        // angle error (signed, -180..+180)
        double error = smallestAngleError(targetAngle, currentAngle);
        double absError = Math.abs(error);

        // ------------------ time & velocity estimation ------------------
        long now = System.currentTimeMillis();
        double dt = (now - lastTimeMs) / 1000.0;
        if (dt <= 1e-3) dt = 1e-3; // guard

        // raw angular velocity (deg/s) measured between last samples (handles wrap)
        double rawVel = smallestAngleError(currentAngle, lastAngle) / dt;

        // filtered velocity (low-pass) to reduce noise and derivative kick
        lastFilteredVelocity = VELOCITY_FILTER_ALPHA * lastFilteredVelocity + (1.0 - VELOCITY_FILTER_ALPHA) * rawVel;

        // save for next call
        lastAngle = currentAngle;
        lastTimeMs = now;

        // ------------------ integrator (optional) ------------------
        // Only accumulate when reasonably far from zero to avoid windup on chatter
        if (absError > (DEAD_BAND * 2)) {
            integrator += error * dt;
            // clamp integrator to avoid huge I-term
            integrator = Range.clip(integrator, -INTEGRATOR_MAX, INTEGRATOR_MAX);
        } else {
            // small decay to remove residual I-term near target
            integrator *= 0.8;
        }

        // ------------------ arrival check ------------------
        // if within deadband and almost stopped -> arrived
        if (absError < DEAD_BAND && Math.abs(lastFilteredVelocity) < STOP_VELOCITY) {
            atTargetLatched = true;
            // don't actively push across zero; hold with zero power (servo friction + small brake)
            return 0.0;
        }
        atTargetLatched = false;

        // ------------------ base PID terms ------------------
        double pTerm = kP * error;
        double iTerm = kI * integrator;
        double dTerm = -kD * lastFilteredVelocity; // use neg sign because velocity is currentAngle - lastAngle

        double pid = pTerm + iTerm + dTerm;

        // ------------------ zone logic ------------------
        double power;

        if (absError > SLOW_ZONE_DEG) {
            // Far: full-fast move. Use a clipped PID that will essentially be full-power
            // but we keep pid so a too-large D can still damp.
            power = Range.clip(pid, -MAX_POWER, MAX_POWER);
            // if PID is too small (tiny gains), force a fast move to avoid sluggishness:
            if (Math.abs(power) < (0.6 * MAX_POWER)) {
                power = Math.signum(error) * 0.9 * MAX_POWER;
            }
        } else {
            // Inside slow zone: progressively scale the PID output toward zero.
            // Use a quadratic shaping so we slow faster near target.
            double frac = Math.max(0.0, Math.min(1.0, absError / SLOW_ZONE_DEG));
            double shape = frac * frac; // quadratic

            power = pid * shape;

            // Ensure we can still overcome static friction (small nudge toward target)
            if (Math.abs(power) < MIN_APPROACH_POWER) {
                power = MIN_APPROACH_POWER * Math.signum(error);
            }

            // Cap the slow-zone power so braking is controllable
            power = Range.clip(power, -MAX_SLOW_POWER, MAX_SLOW_POWER);

            // Additional damping: if velocity is large and heading toward the target,
            // reduce power more aggressively to avoid overshoot.
            if (Math.signum(error) == Math.signum(lastFilteredVelocity)) {
                // moving away from target (velocity has same sign as error) -> be aggressive
                double velScale = Math.min(1.0, Math.abs(lastFilteredVelocity) / 120.0); // 0..1
                power *= (1.0 - 0.5 * velScale); // reduce up to 50% when very fast
            }
        }

        // ------------------ final saturate & return ------------------
        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        // populate telemetry (so you can see debug while tuning)
        t.pidP = pTerm;
        t.pidI = iTerm;
        t.pidD = dTerm;

        return power;
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

            // find nearest silo index for that angle (with hysteresis fallback)
            int siloIndex = angleToSilo(sensorAngle);
            if (siloIndex == -1) {
                int prev = lastSeenSiloBySensor[sensorIdx];
                if (prev != -1) {
                    double prevErr = Math.abs(smallestAngleError(SILO_ANGLES[prev], sensorAngle));
                    if (prevErr < ANGLE_TOLERANCE * 3.0) siloIndex = prev;
                }
            }

            if (siloIndex != -1) {
                // **THIS IS THE IMPORTANT PART**: write the sensor reading into the silo slot
                silos[siloIndex] = detectBall(s, sensorIdx);
                lastSeenSiloBySensor[sensorIdx] = siloIndex;
            } else {
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
        // allow fast sign reversal (larger allowed delta) when reversing direction
        double allowedDelta = MAX_DELTA;
        if (Math.signum(p) != Math.signum(lastPower)) {
            allowedDelta *= 4.0; // allow quicker reversal to avoid overshoot
        }
        if (Math.abs(diff) > allowedDelta) {
            p = lastPower + Math.signum(diff) * allowedDelta;
        }
        lastPower = p;
        return clamp(p, -1, 1);
    }

    // ─────────────────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────────────────
    private static double normalize(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private boolean atTarget() {
        return atTargetLatched;
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
            if (b > 250 && b > g) return BallColor.PURPLE;
            else if (g > 150) return BallColor.GREEN;
            return BallColor.NONE;
        } else if (sensorIndex == 1) { // backLeft
            if (b > 225  && b > g) return BallColor.PURPLE;
            else if (g > 350) return BallColor.GREEN;
            return BallColor.NONE;
        } else { // sensorIndex == 2 -> backRight
            if (b > 175  && b > g) return BallColor.PURPLE;
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
            if (Math.abs(error) < ANGLE_TOLERANCE_COLOR) return i;
        }
        return -1;
    }

    public void enableSensorUpdates() {
        sensorUpdatesEnabled = true;
    }

    public void disableSensorUpdates() {
        sensorUpdatesEnabled = false;
    }

    public void goToSilo(int index) {
        switch (index) {
            case 0: goToSilo1(); break;
            case 1: goToSilo2(); break;
            case 2: goToSilo3(); break;
        }
    }

    public void initSilos() {
        silo1 = normalize(Parameters.spinLocation - 120);
        silo2 = normalize(Parameters.spinLocation);
        silo3 = normalize(Parameters.spinLocation + 120);

        // keep your existing power limits and deadband (tweakable)
        SILO_ANGLES = new double[]{
                silo1,   // SILO_1
                silo2,   // SILO_2
                silo3    // SILO_3
        };
    }

}
