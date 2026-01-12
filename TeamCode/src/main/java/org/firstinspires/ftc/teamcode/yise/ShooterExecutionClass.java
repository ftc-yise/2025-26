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
    // add near top of class
    private static final int LIFTER_DOWN_POS = 0;   // set to your measured down encoder count
    private static final int LIFTER_UP_POS   = 800; // set to your measured up encoder count
    private final lifter lifter;
    private final Spindexer spindexer;
    private final ShooterClass shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private final double LIFTER_MOVE_TIMEOUT = 1.2; // seconds


    private int shotsFired = 0;
    private int totalShots = 0;        // dynamically computed at cycle start
    public int currentSiloIndex = -1; // currently active silo
    public boolean jittered = false;

    private final double LIFT_UP = 0.75;
    private final double LIFT_DOWN = 0.00;

    // --- Placeholder for future pattern logic ---
    // You could add an array of integers, or a list of Silo/BallColor targets
    // and iterate through them here. For now, we leave it null.
    // Example:
    // private int[] pattern = null;

    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwareMap, lifter lift) {
        this.spindexer = spin;
        this.shooter = shooter;
        this.lifter = lift;
        lifter.setPresetPositions(0.0, 1.0);
        lifter.setCalibration(1.978, 0.1, 3.77, 1.1);  //Jack updating the lifter arm height (V2: was 3.758)

    }

    // ---------------- START CYCLE ----------------
    public void startCycle() {
        if (state != State.IDLE) return;
        totalShots = 0;
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        for (Spindexer.BallColor color : colors) {
            if (color != Spindexer.BallColor.NONE) totalShots++;
        }


        // inside startCycle() after computing totalShots:
        if (totalShots == 0) {
            // If we've never jittered this activation, do a short jitter to settle balls.
            if (!jittered) {
                jittered = true;
                // disable sensor updates while we're jittering to avoid false reads
                spindexer.disableSensorUpdates();
                // reset the timer so the JITTER timing is reliable
                timer.reset();
                // enter jitter state
                state = State.JITTER;
            } else {
                // we already jittered — nothing to do, re-enable sensors and go idle
                spindexer.enableSensorUpdates();
                state = State.IDLE;
            }
            return;
        }
        // nothing to do


        shotsFired = 0;
        state = State.MOVE_TO_SILO;

        // Move to first non-empty silo
        moveToNextFullSilo();
        timer.reset();
    }

    // ---------------- UPDATE LOOP ----------------
    public void update() {
        lifter.update();

        switch (state) {
            case JITTER:
                // deterministic 240ms two-pulse jitter: forward 0.12s, reverse 0.12s, stop
                double t = timer.seconds();
                if (t < 0.22) {
                    // small forward pulse — tune amplitude if needed (0.4 is a good start)
                    spindexer.setManual(0.3);
                } else if (t < 0.44) {
                    // short reverse pulse
                    spindexer.setManual(-0.2);
                } else {
                    // finish jitter: stop motor, re-enable sensing, clear timers and go idle
                    spindexer.setManual(0.0);
                    spindexer.enableSensorUpdates();
                    spindexer.goToSilo1();
                    if (t > 1) {
                        timer.reset();
                        state = State.COMPLETE;
                    }
                }
                return;


            case IDLE:
                shooter.update(false, false, false);
                return;

            case MOVE_TO_SILO:
                if (Math.abs(spindexer.getTelemetry().angleError) < 0.81) { // loosen tolerance
                    spindexer.sampleSensorsNow();
                    state = State.SPIN_WAIT;
                    timer.reset();
                } else if (timer.seconds() > 1.5) { // ⏱ watchdog
                    spindexer.sampleSensorsNow();
                    state = State.SPIN_WAIT;
                    timer.reset();
                }
                break;


            case SPIN_WAIT:
                if (timer.seconds() > 0.3) {
                    state = State.SPIN_UP_SHOOTER;
                    timer.reset();
                }
                break;

            case SPIN_UP_SHOOTER:
                if (timer.seconds() > 1.0) { // give shooter time to spin up
                    lifter.setUp();
                    timer.reset();
                    state = State.FIRE_LIFT_UP;
                }
                break;

            case FIRE_LIFT_UP:
                if (lifter.isUp()  || timer.seconds() > LIFTER_MOVE_TIMEOUT) {
                    lifter.setDown();
                    timer.reset();
                    state = State.FIRE_LIFT_DOWN;
                }
                break;

            case FIRE_LIFT_DOWN:
                if (lifter.isDown()) {
                    shotsFired++;

                    // Clear the fired silo
                    if (currentSiloIndex != -1) {
                        spindexer.clearSilo(currentSiloIndex);
                    }

                    if (shotsFired >= totalShots) {
                        state = State.COMPLETE;
                    } else {
                        state = State.NEXT_SILO;
                    }
                }
                break;

            case NEXT_SILO:
                moveToNextFullSilo();
                timer.reset();
                state = State.MOVE_TO_SILO;
                break;

            case COMPLETE:
                spindexer.enableSensorUpdates();
                spindexer.sampleSensorsNow();
                state = State.IDLE;
                break;

        }
    }

    // ---------------- HELPER: Move to next non-empty silo ----------------
    private void moveToNextFullSilo() {
        spindexer.sampleSensorsNow();
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        int nextIndex = (currentSiloIndex + 1) % 3;

        // Find the next silo with a ball
        for (int i = 0; i < 3; i++) {
            int idx = (nextIndex + i) % 3;
            if (colors[idx] != Spindexer.BallColor.NONE) {
                currentSiloIndex = idx;

                // Move spindexer to that silo
                switch (idx) {
                    case 0: spindexer.goToSilo1(); break;
                    case 1: spindexer.goToSilo2(); break;
                    case 2: spindexer.goToSilo3(); break;
                }

                // --- PLACEHOLDER FOR FUTURE PATTERN LOGIC ---
                // Example: if you want to skip certain silos, or follow a pre-defined order,
                // you can check your pattern array here and override the currentSiloIndex.

                return;
            }
        }

        // If none found (should not happen), stay idle
        currentSiloIndex = -1;
        state = State.COMPLETE;
    }


    // ---------------- IS BUSY ----------------
    public boolean isBusy() {
        return state != State.IDLE;
    }
    // -------------------------------------------------------------
    //  HELPER — Move to next silo in order 1→2→3→1
    // -------------------------------------------------------------

}
