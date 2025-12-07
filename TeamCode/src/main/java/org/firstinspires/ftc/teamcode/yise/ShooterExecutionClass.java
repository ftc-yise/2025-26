package org.firstinspires.ftc.teamcode.yise;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
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

    // Lift servo controlled from OpMode, so we store a pointer

    // Lift positions
    private final double LIFT_UP = 0.75;
    private final double LIFT_DOWN = 0.00;


    public ShooterExecutionClass(Spindexer spin, ShooterClass shooter, HardwareMap hardwaremap) {
        this.spindexer = spin;
        this.shooter = shooter;
        lift = hardwaremap.get(Servo.class, "lift");
    }

    // -------------------------------------------------------------
    //  START 3-SHOT AUTOMATIC SEQUENCE
    // -------------------------------------------------------------
    public void startCycle() {
        if (state != State.IDLE) return;

        shotsFired = 0;
        state = State.MOVE_TO_SILO;

        // Move to first silo (whatever is next)
        moveSpindexerToNextSilo();

        timer.reset();
    }

    // -------------------------------------------------------------
    //  MAIN UPDATE — CALL EVERY LOOP
    // -------------------------------------------------------------
    public void update() {

        switch (state) {

            case IDLE:
                shooter.update(false, false, false); // Y = FULL
                return;

            case MOVE_TO_SILO:
                if (spindexer.getTelemetry().angleError < 3) {
                    state = State.SPIN_WAIT;
                    timer.reset();
                }
                break;

            case SPIN_WAIT:
                // small settling delay
                if (timer.seconds() > 0.1) {
                    state = State.SPIN_UP_SHOOTER;
                    timer.reset();
                }
                break;

            case SPIN_UP_SHOOTER:
                // give shooter time to reach speed
                if (shotsFired >= 2){
                    if (timer.seconds() > 0.35) {
                        lift.setPosition(Servo.MAX_POSITION);
                        timer.reset();
                        state = State.FIRE_LIFT_UP;
                    }
                } else {
                    if (timer.seconds() > 0.55) {
                        lift.setPosition(Servo.MAX_POSITION);
                        timer.reset();
                        state = State.FIRE_LIFT_UP;
                    }
                }
                break;

            case FIRE_LIFT_UP:
                    if (timer.seconds() > 0.35) {
                        lift.setPosition(Servo.MIN_POSITION);
                        timer.reset();
                        state = State.FIRE_LIFT_DOWN;
                    }
                break;

            case FIRE_LIFT_DOWN:
                if (timer.seconds() > 0.45) {
                    shotsFired++;

                    if (shotsFired >= 3) {
                        state = State.COMPLETE;
                    } else {
                        state = State.NEXT_SILO;
                    }
                }
                break;

            case NEXT_SILO:
                moveSpindexerToNextSilo();
                timer.reset();
                state = State.MOVE_TO_SILO;
                break;

            case COMPLETE:
                state = State.IDLE;
                break;
        }
    }

    // -------------------------------------------------------------
    //  HELPER — Move to next silo in order 1→2→3→1
    // -------------------------------------------------------------
    private void moveSpindexerToNextSilo() {
        switch (spindexer.mode) {
            case SILO_1:
                spindexer.goToSilo2();
                break;
            case SILO_2:
                spindexer.goToSilo3();
                break;
            default:
                spindexer.goToSilo1();
                break;
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }
}
