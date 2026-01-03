package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterExecutionClass {

    public enum State {
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
    private Servo lift;
    private final Spindexer spindexer;
    private final ShooterClass shooter;
    private final ElapsedTime timer = new ElapsedTime();

    private int shotsFired = 0;
    private int totalShots = 0;        // dynamically computed at cycle start
    public int currentSiloIndex = -1; // currently active silo

    private final double LIFT_UP = 0.75;
    private final double LIFT_DOWN = 0.00;

    // --- Placeholder for future pattern logic ---
    // You could add an array of integers, or a list of Silo/BallColor targets
    // and iterate through them here. For now, we leave it null.
    // Example:
    // private int[] pattern = null;

    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwareMap) {
        this.spindexer = spin;
        this.shooter = shooter;
        lift = hardwareMap.get(Servo.class, "lift");
    }

    // ---------------- START CYCLE ----------------
    public void startCycle() {
        if (state != State.IDLE) return;

        // Compute which silos actually have balls
        spindexer.disableSensorUpdates(); //
        totalShots = 0;
        Spindexer.BallColor[] colors = spindexer.getTelemetry().siloColors;
        for (Spindexer.BallColor color : colors) {
            if (color != Spindexer.BallColor.NONE) totalShots++;
        }


        if (totalShots == 0) return; // nothing to do

        shotsFired = 0;
        state = State.MOVE_TO_SILO;

        // Move to first non-empty silo
        moveToNextFullSilo();
        timer.reset();
    }

    // ---------------- UPDATE LOOP ----------------
    public void update() {

        switch (state) {

            case IDLE:
                shooter.update(false, false, false);
                return;

            case MOVE_TO_SILO:
                if (Math.abs(spindexer.getTelemetry().angleError) < 1.0) {
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
                    lift.setPosition(Servo.MAX_POSITION);
                    timer.reset();
                    state = State.FIRE_LIFT_UP;
                }
                break;

            case FIRE_LIFT_UP:
                if (timer.seconds() > 0.45) {
                    lift.setPosition(Servo.MIN_POSITION);
                    timer.reset();
                    state = State.FIRE_LIFT_DOWN;
                }
                break;

            case FIRE_LIFT_DOWN:
                if (timer.seconds() > 0.45) {
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
                spindexer.enableSensorUpdates(); // 🟢 restore sensing
                state = State.IDLE;
                break;
        }
    }

    // ---------------- HELPER: Move to next non-empty silo ----------------
    private void moveToNextFullSilo() {
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
}
