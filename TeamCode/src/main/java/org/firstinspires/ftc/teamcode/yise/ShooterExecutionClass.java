package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ShooterExecutionClass
 *
 * Handles:
 *  - Pattern-based shooting (3-ball pattern repeated up to 9)
 *  - Fastest fallback shooting if pattern breaks
 *  - Shot logging (9 slots)
 *  - Safe recovery if wrong ball is inserted
 */
public class ShooterExecutionClass {

    // ─────────────────────────────────────────────
    // SUBSYSTEMS
    // ─────────────────────────────────────────────
    private final Spindexer spindexer;
    private final ShooterClass shooter;
    private final lifter lifter;

    // ─────────────────────────────────────────────
    // STATE MACHINE
    // ─────────────────────────────────────────────
    private enum State {
        IDLE,
        SELECT_SILO,
        WAIT_FOR_RPM,
        LIFT,
        FEED,
        RESET
    }

    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    // ─────────────────────────────────────────────
    // PATTERN + LOGGING
    // ─────────────────────────────────────────────
    private final int[] shotLog = new int[9];
    private int shotCount = 0;

    private int[] pattern = null;   // length = 3
    private int patternIndex = 0;
    private boolean usePattern = false;

    private int currentSilo = -1;

    // ─────────────────────────────────────────────
    // TIMING CONSTANTS
    // ─────────────────────────────────────────────
    private static final double RPM_TIMEOUT = 0.8;
    private static final double FEED_TIME   = 0.25;
    private static final double RESET_TIME  = 0.2;

    // ─────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────
    public ShooterExecutionClass(
            Spindexer spindexer,
            ShooterClass shooter,
            lifter lifter
    ) {
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.lifter = lifter;

        lifter.setPresetPositions(0.0, 1.0);
        lifter.setCalibration(0.7, 0, 1.6, 1);

        clearShotLog();
    }

    // ─────────────────────────────────────────────
    // PATTERN API
    // ─────────────────────────────────────────────

    /**
     * Pattern must be length 3.
     * Example: {0,1,2}
     */
    public void setPattern(int[] pattern) {
        if (pattern == null || pattern.length != 3) {
            this.pattern = null;
            usePattern = false;
            return;
        }

        this.pattern = pattern.clone();
        this.patternIndex = 0;
        this.usePattern = true;
    }

    public void clearShotLog() {
        for (int i = 0; i < shotLog.length; i++) {
            shotLog[i] = -1;
        }
        shotCount = 0;
        patternIndex = 0;
    }

    public int[] getShotLog() {
        return shotLog.clone();
    }

    // ─────────────────────────────────────────────
    // CONTROL
    // ─────────────────────────────────────────────
    public void startCycle() {
        if (state != State.IDLE) return;

        spindexer.sampleSensorsNow();
        timer.reset();
        state = State.SELECT_SILO;
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    // ─────────────────────────────────────────────
    // MAIN UPDATE LOOP
    // ─────────────────────────────────────────────
    public void update() {

        switch (state) {

            case IDLE:
                shooter.update(false,false,false);
                return;

            // ─────────────────────────────────────
            case SELECT_SILO:
                currentSilo = chooseNextSilo();

                if (currentSilo == -1) {
                    state = State.IDLE;
                    return;
                }

                spindexer.goToSilo(currentSilo);
                timer.reset();
                state = State.WAIT_FOR_RPM;
                break;

            // ─────────────────────────────────────
            case WAIT_FOR_RPM:
                lifter.setDown();
                spindexer.disableSensorUpdates();

                if (shooter.getTelemetry().readyLoose ||
                        timer.seconds() > RPM_TIMEOUT) {

                    lifter.setUp();
                    timer.reset();
                    state = State.LIFT;
                }
                break;

            // ─────────────────────────────────────
            case LIFT:
                if (lifter.getTelemetry().position > 0.85 ||
                        timer.seconds() > 0.35) {

                    spindexer.enableSensorUpdates();
                    spindexer.setManual(0.15);
                    timer.reset();
                    state = State.FEED;
                }
                break;

            // ─────────────────────────────────────
            case FEED:
                if (timer.seconds() > FEED_TIME) {

                    spindexer.stop();
                    lifter.setDown();

                    logShot(currentSilo);
                    spindexer.clearSilo(currentSilo);

                    timer.reset();
                    state = State.RESET;
                }
                break;

            // ─────────────────────────────────────
            case RESET:
                if (lifter.isDown() && timer.seconds() > RESET_TIME) {
                    state = State.SELECT_SILO;
                }
                break;
        }
    }

    // ─────────────────────────────────────────────
    // SILO SELECTION LOGIC
    // ─────────────────────────────────────────────
    private int chooseNextSilo() {

        spindexer.sampleSensorsNow();
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;

        // Disable pattern permanently after 9 shots
        if (shotCount >= 9) {
            usePattern = false;
        }

        // ───── Pattern path ─────
        if (usePattern && pattern != null) {
            int desired = pattern[patternIndex % 3];

            if (colors[desired] != Spindexer.BallColor.NONE) {
                patternIndex++;
                return desired;
            }

            // Pattern broken → fallback
            usePattern = false;
        }

        // ───── Fastest fallback ─────
        for (int i = 0; i < 3; i++) {
            if (colors[i] != Spindexer.BallColor.NONE) {
                return i;
            }
        }

        return -1;
    }

    // ─────────────────────────────────────────────
    // LOGGING
    // ─────────────────────────────────────────────
    private void logShot(int siloIndex) {
        if (shotCount < shotLog.length) {
            shotLog[shotCount] = siloIndex;
        }
        shotCount++;
    }
}
