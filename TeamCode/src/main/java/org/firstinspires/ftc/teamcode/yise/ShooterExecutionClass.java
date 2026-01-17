package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterExecutionClass {

    public enum State {
        JITTER,
        IDLE,
        MOVE_TO_SILO,
        SPIN_WAIT,
        SPIN_UP_SHOOTER,
        FIRE_LIFT_UP,
        FIRE_LIFT_DOWN,
        NEXT_SILO,
        COMPLETE
    }

    private State state = State.IDLE;

    private final lifter lifter;
    private final Spindexer spindexer;
    private final ShooterClass shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private final double LIFTER_MOVE_TIMEOUT = 4.2; // seconds

    private int shotsFired = 0;
    private int totalShots = 0;        // dynamically computed at cycle start
    public int currentSiloIndex = -1; // currently active silo
    public boolean jittered = false;

    // --- Force-mode flag (temporary override) ---
    public boolean forceShooting = false;

    // --- simple constants ---
    private final double LIFT_UP = 0.75;
    private final double LIFT_DOWN = 0.00;

    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwareMap, lifter lift) {
        this.spindexer = spin;
        this.shooter = shooter;
        this.lifter = lift;
        // keep previous default init (optional)
        lifter.setPresetPositions(0.0, 1.0);
        lifter.setCalibration(.7, 0, 1.6, 1);
    }

    // ---------------- START CYCLE ----------------
    public void startCycle() {
        if (state != State.IDLE) return;

        // Compute which silos actually have balls
        spindexer.disableSensorUpdates();
        totalShots = 0;
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        for (Spindexer.BallColor color : colors) {
            if (color != Spindexer.BallColor.NONE) totalShots++;
        }

        if (totalShots == 0) {
            if (!jittered) {
                // jitter once to settle
                jittered = true;
                timer.reset();
                state = State.JITTER;
            } else {
                spindexer.enableSensorUpdates();
                state = State.IDLE;
            }
            return;
        }

        shotsFired = 0;
        state = State.MOVE_TO_SILO;
        moveToNextFullSilo();
        timer.reset();
    }

    // ---------------- FORCED MODE API ----------------
    // Call this to start firing continuously (ignores sensor occupancy)
    public void startForcedCycle() {
        if (state != State.IDLE) {
            // If already running, do nothing
            forceShooting = true;
            return;
        }
        forceShooting = true;
        spindexer.disableSensorUpdates(); // avoid conflicting sensor updates
        shotsFired = 0;
        totalShots = Integer.MAX_VALUE; // effectively "until stopped"
        // Start from next (or 0) silo so mechanism cycles
        currentSiloIndex = 0;
        spindexer.goToSilo1();
        state = State.MOVE_TO_SILO;
        timer.reset();
    }

    // Stop forced-fire and re-enable normal behavior
    public void stopForcedCycle() {
        forceShooting = false;
        spindexer.enableSensorUpdates();
        // gracefully finish this cycle (let update() put us back to IDLE)
        state = State.COMPLETE;
    }

    // ---------------- UPDATE LOOP ----------------
    public void update() {
        lifter.update();

        switch (state) {
            case JITTER: {
                double t = timer.seconds();
                if (t < 0.22) {
                    spindexer.setManual(0.3);
                } else if (t < 0.44) {
                    spindexer.setManual(-0.2);
                } else {
                    spindexer.setManual(0.0);
                    spindexer.enableSensorUpdates();
                    spindexer.goToSilo1();
                    if (t > 1) {
                        timer.reset();
                        state = State.COMPLETE;
                    }
                }
                return;
            }

            case IDLE:
                shooter.update(false, false, false);
                return;

            case MOVE_TO_SILO:
                // If forced, accept looser tolerance and keep moving between silos
                double angleErr = Math.abs(spindexer.getTelemetry().angleError);
                if (timer.seconds() > 0.12) {
                    if (angleErr < 3) {
                        spindexer.sampleSensorsNow();
                        state = State.SPIN_WAIT;
                        timer.reset();
                    } else if (timer.seconds() > 1.3) { // watchdog
                        spindexer.sampleSensorsNow();
                        state = State.SPIN_WAIT;
                        timer.reset();
                    }
                }
                break;


            case SPIN_WAIT:
                if (timer.seconds() > .25) {
                    state = State.SPIN_UP_SHOOTER;
                    timer.reset();
                }
                break;

            case SPIN_UP_SHOOTER:
                if (shooter.getTelemetry().errorRPM < 100) {
                    if (timer.seconds() > .35) {
                        lifter.setUp();
                        timer.reset();
                        state = State.FIRE_LIFT_UP;
                    }
                }
                break;

            case FIRE_LIFT_UP:
                if (lifter.isUp() || timer.seconds() > LIFTER_MOVE_TIMEOUT) {
                    if (timer.seconds() > .2) {
                        lifter.setDown();
                        timer.reset();
                        state = State.FIRE_LIFT_DOWN;
                    }
                }
                break;

            case FIRE_LIFT_DOWN:
                // On forced mode we don't decrement totalShots; we only stop when user calls stopForcedCycle()
                if (lifter.isDown() || timer.seconds() > (LIFTER_MOVE_TIMEOUT + 0.3)) {
                    shotsFired++;

                    // Clear the fired silo only if not in force-mode (avoid hiding state)
                    if (!forceShooting && currentSiloIndex != -1) {
                        spindexer.clearSilo(currentSiloIndex);
                    }

                    if (!forceShooting) {
                        if (shotsFired >= totalShots) {
                            state = State.COMPLETE;
                        } else {
                            state = State.NEXT_SILO;
                        }
                    } else {
                        // forced -> continue cycling
                        state = State.NEXT_SILO;
                    }
                }
                break;

            case NEXT_SILO:
                moveToNextFullSilo(); // this function has been made forced-aware below
                timer.reset();
                state = State.MOVE_TO_SILO;
                break;

            case COMPLETE:
                // restore sensor updates (if not forced)
                if (!forceShooting) spindexer.enableSensorUpdates();
                spindexer.sampleSensorsNow();
                state = State.IDLE;
                break;
        }
    }

    // ---------------- HELPER: Move to next silo (forced-aware) ----------------
    private void moveToNextFullSilo() {
        if (forceShooting) {
            // blind-cycle: simply step to next index and go there
            currentSiloIndex = (currentSiloIndex + 1) % 3;
            switch (currentSiloIndex) {
                case 0: spindexer.goToSilo1(); break;
                case 1: spindexer.goToSilo2(); break;
                case 2: spindexer.goToSilo3(); break;
            }
            return;
        }

        // normal behavior: pick next non-empty silo (keeps existing API)
        spindexer.sampleSensorsNow();
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        int nextIndex = (currentSiloIndex + 1) % 3;

        for (int i = 0; i < 3; i++) {
            int idx = (nextIndex + i) % 3;
            if (colors[idx] != Spindexer.BallColor.NONE) {
                currentSiloIndex = idx;
                switch (idx) {
                    case 0: spindexer.goToSilo1(); break;
                    case 1: spindexer.goToSilo2(); break;
                    case 2: spindexer.goToSilo3(); break;
                }
                return;
            }
        }

        // none found -> finish
        currentSiloIndex = -1;
        state = State.COMPLETE;
    }

    // ---------------- IS BUSY ----------------
    public boolean isBusy() {
        return state != State.IDLE;
    }

}
