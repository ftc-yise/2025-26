package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.bylazar.configurables.annotations.Configurable;

@Configurable // Exposes public static fields for tuning via your configuration library
public class Turret {

    // --- PID TUNING CONSTANTS (Adjust these via your tuning interface!) ---
    // Turret uses Limelight's 'ty' (vertical angle) as its input error,
    // assuming a vertically-mounted camera where 'ty' measures horizontal deviation.
    public static double kP = 0.05;      // Proportional gain (start tuning here)
    public static double kI = 0.0;       // Integral gain (tune last, if needed)
    public static double kD = 0.001;     // Derivative gain (for dampening)
    public static double MAX_POWER = 0.45; // Max motor power
    public static double MIN_POWER_FLOOR = 0.15; // Minimum power to overcome friction/inertia
    public static double INTEGRAL_MAX = 5.0; // Anti-windup limit for integral term
    public static double TARGET_TOLERANCE_DEG = 1.0; // Turret is "centered" when |ty| < tolerance

    // --- PID STATE VARIABLES ---
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTime = System.currentTimeMillis();

    // --- Existing Variables ---
    LLResult result = null;
    public double turretPower = 0.0;
    // Note: myTy will now store the error, which is the Limelight's Ty value.
    public double myTy = 0.0;
    public DcMotor turret;
    public Limelight3A limelight;
    public Telemetry telemetry;
    public enum turretAlliance{
        RED,
        BLUE,
    }
    public enum turretDirection {
        LEFT,
        RIGHT,
        STOP,
    }
    public enum turretMode{
        AUTO,
        MANUAL,
    }
    public turretMode mode;

    public Turret (HardwareMap hardwareMap, turretAlliance alliance, Telemetry telem){
        telemetry = telem;

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        if (alliance == turretAlliance.RED) {
            limelight.pipelineSwitch(4);
        }else if (alliance== turretAlliance.BLUE) {
            limelight.pipelineSwitch(3);
        }
        mode = turretMode.MANUAL;

        lastTime = System.currentTimeMillis();
    }

    public void changeMode() {
        if (mode == turretMode.AUTO) {
            mode = turretMode.MANUAL;
            stop();
            resetPID();
        } else if (mode == turretMode.MANUAL) {
            mode = turretMode.AUTO;
        }
    }

    public void manualMode(turretDirection direction) {
        mode = turretMode.MANUAL;

        switch (direction) {
            case LEFT:
                turret.setPower(-0.4);
                break;
            case RIGHT:
                turret.setPower(0.4);
                break;
            case STOP:
                turret.setPower(0);
                break;
        }
    }

    // --- UPDATED AutoMode() using Limelight 'ty' ---
    public void autoMode(){
        mode = turretMode.AUTO;

        result = limelight.getLatestResult();

        // Check if the target is seen (isValid())
        if (result != null && result.isValid()) {
            // Use 'ty' as the horizontal error input for the PID controller
            double currentError_ty = result.getTy();
            myTy = currentError_ty; // Update the public variable

            turretPower = calculatePIDPower(currentError_ty);

            // Add telemetry for tuning
            telemetry.addData("Turret Mode", "AUTO (PID)");
            telemetry.addData("Error (ty)", currentError_ty);
            telemetry.addData("Turret Power", turretPower);

        } else {
            turretPower = 0;
            resetPID();
            telemetry.addData("Turret Mode", "AUTO (Target Lost)");
            telemetry.addData("Turret Power", 0);
        }

        turret.setPower(turretPower);
    }

    public void stop(){
        turret.setPower(0);
    }

    /**
     * Resets the integral and derivative components of the PID controller.
     */
    private void resetPID() {
        integralSum = 0.0;
        lastError = 0.0;
        lastTime = System.currentTimeMillis();
    }

    /**
     * Calculates the motor power using the PID algorithm.
     * @param currentError The horizontal error (Limelight ty) in degrees.
     * @return The calculated motor power (-MAX_POWER to MAX_POWER).
     */
    private double calculatePIDPower(double currentError) {
        // If the error is within tolerance, stop and reset
        if (Math.abs(currentError) < TARGET_TOLERANCE_DEG) {
            resetPID();
            return 0.0;
        }

        long currentTime = System.currentTimeMillis();
        // Time in seconds for accurate integral and derivative
        double deltaTime = (currentTime - lastTime) / 1000.0;

        // 1. Proportional Term (P)
        double proportional = kP * currentError;

        // 2. Integral Term (I)
        // Only integrate when somewhat close to prevent windup
        if (Math.abs(currentError) < 10.0) {
            integralSum += currentError * deltaTime;
        } else {
            integralSum = 0; // Clear integral if error is too large
        }

        // Anti-windup: Limit the integral sum
        if (integralSum > INTEGRAL_MAX) {
            integralSum = INTEGRAL_MAX;
        } else if (integralSum < -INTEGRAL_MAX) {
            integralSum = -INTEGRAL_MAX;
        }
        double integral = kI * integralSum;

        // 3. Derivative Term (D)
        // Prevent division by zero on the first call
        double derivative = (deltaTime != 0) ? kD * ((currentError - lastError) / deltaTime) : 0.0;

        // 4. Combine terms
        double outputPower = proportional + integral + derivative;

        // 5. Apply power floor (only applies power if movement is needed)
        // This makes sure we overcome static friction
        if (outputPower > 0 && Math.abs(outputPower) < MIN_POWER_FLOOR) {
            outputPower = MIN_POWER_FLOOR;
        } else if (outputPower < 0 && Math.abs(outputPower) < MIN_POWER_FLOOR) {
            outputPower = -MIN_POWER_FLOOR;
        }

        // 6. Clamp the power to the motor's absolute limits
        if (outputPower > MAX_POWER) {
            outputPower = MAX_POWER;
        } else if (outputPower < -MAX_POWER) {
            outputPower = -MAX_POWER;
        }

        // 7. Update state for the next loop
        lastError = currentError;
        lastTime = currentTime;

        return outputPower;
    }
}