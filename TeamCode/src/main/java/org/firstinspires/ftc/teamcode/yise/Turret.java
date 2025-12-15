package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Turret {

    // --- PD TUNING CONSTANTS (These still need adjusted) ---
    public static double kP = 0.05; // Proportional Gain: Scales the power output based on the current error.
    public static double kD = 0.05; // Derivative Gain: Damps sudden changes in error, reducing overshoot/wobbling.

    public static double MAX_POWER = 0.7; // Hard limit on the maximum power the turret motor receives.
    public static double MIN_POWER_FLOOR = 0.25; // Minimum required power to overcome static friction and start motor movement.
    public static double TARGET_TOLERANCE_DEG = 0.8; // The error threshold (in degrees) at which the turret considers itself "on target" and stops.

    // --- FINAL SIGN CHECK MULTIPLIER ---
    /**
     * This multiplier cancels out any persistent sign mismatch between the motor's
     * physical direction and the Limelight's coordinate system. This is what needs changed if the turret avoids the Apriltag
     * Set to 1.0 or -1.0 in the dashboard to ensure the turret moves TOWARD the target.
     */
    public static double FINAL_DIRECTION_MULTIPLIER = 1.0;

    // --- PD STATE VARIABLES ---
    private double lastError = 0.0; // Stores the error from the previous loop iteration for calculating the derivative.
    private long lastTime; // Stores the time of the previous loop iteration.

    {
        System.currentTimeMillis();
    }

    // --- CLASS VARIABLES ---
    LLResult result = null; // Stores the latest data retrieved from the Limelight.
    public double turretPower = 0.0; // The calculated power applied to the motor.
    public double myTy = 0.0; // The horizontal error (ty) from the Limelight, after sign correction.
    public DcMotor turret; // Hardware object for the turret motor.
    public Limelight3A limelight; // Hardware object for the Limelight camera.
    public Telemetry telemetry; // Telemetry object for displaying data on the driver station.

    // Enumerations (Enums) to clearly define state and intent
    public enum turretAlliance{RED, BLUE,}
    public enum turretDirection {LEFT, RIGHT, STOP,}
    public enum turretMode{AUTO, MANUAL,}
    public turretMode mode; // Current operating mode of the turret.

    /**
     * Turret class constructor. Initializes all hardware and settings.
     */
    public Turret (HardwareMap hardwareMap, turretAlliance alliance, Telemetry telem){
        telemetry = telem;

        // Initialize motor hardware
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Use direct power control

        // DO NOT CHANGE THIS, instead change Final Direction Multiplier up above
        turret.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Limelight hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Set high poll rate for fast targeting
        limelight.start();

        // Set the appropriate Limelight pipeline based on alliance color
        if (alliance == turretAlliance.RED) {
            limelight.pipelineSwitch(4);
        }else if (alliance== turretAlliance.BLUE) {
            limelight.pipelineSwitch(3);
        }
        mode = turretMode.MANUAL; // Start in manual mode

        lastTime = System.currentTimeMillis();
    }

    /**
     * Controls the turret based on driver input (used in MANUAL mode).
     */
    public void manualMode(turretDirection direction) {
        mode = turretMode.MANUAL;

        switch (direction) {
            case LEFT:
                turret.setPower(0.7);
                break;
            case RIGHT:
                turret.setPower(-0.7);
                break;
            case STOP:
                turret.setPower(0);
                break;
        }
    }

    /**
     * Runs the automatic targeting logic using PID control.
     */
    public void autoMode(){
        mode = turretMode.AUTO;

        result = limelight.getLatestResult(); // Get the newest vision data

        if (result != null && result.isValid()) {
            double rawError_ty = result.getTy(); // Get raw horizontal error

            // 1. Correct Limelight Sign: Multiplies by -1.0 to account for the 90° clockwise flip.
            double currentError = rawError_ty * -1.0;
            myTy = currentError; // Store the corrected error

            // 2. Calculate PD Power: Generates a signed power based on the error
            turretPower = calculatePDPower(currentError);

            // 3. Apply the final sign fix: Multiplies by 1.0 or -1.0 (from dashboard)
            // to correct the final motor direction mismatch.
            turretPower = turretPower * FINAL_DIRECTION_MULTIPLIER;

        } else {
            // Target lost: stop the motor and reset PD state
            turretPower = 0;
            resetPD();
            telemetry.addData("Turret Mode", "AUTO (Target Lost)");
        }

        // Display telemetry and apply final calculated power
        telemetry.addData("Mode", "AUTO (PD Control)");
        telemetry.addData("Final Power", turretPower);
        turret.setPower(turretPower);
    }

    /**
     * Stops the turret motor immediately.
     */
    public void stop(){
        turret.setPower(0);
    }

    /**
     * Resets the state variables (lastError, lastTime) for the PD controller.
     */
    private void resetPD() {
        lastError = 0.0;
        lastTime = System.currentTimeMillis();
    }

    /**
     * Core PD calculation function.
     */
    private double calculatePDPower(double currentError) {
        // 1. Check Tolerance: If error is small enough, stop and reset.
        if (Math.abs(currentError) < TARGET_TOLERANCE_DEG) {
            resetPD();
            return 0.0;
        }

        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds

        // Proportional Term: P = kP * error
        double proportional = kP * currentError;

        // Derivative Term: D = kD * (change_in_error / delta_time)
        double derivative = 0.0;
        if (deltaTime != 0) {
            derivative = kD * ((currentError - lastError) / deltaTime);
        }

        // 3. Combine P and D
        double outputPower = proportional + derivative;

        // 4. Min Power Floor (Friction Compensation)
        // If the calculated power is low, boost it up to MIN_POWER_FLOOR magnitude
        // to ensure the motor starts moving.
        if (outputPower > 0 && Math.abs(outputPower) < MIN_POWER_FLOOR) {
            outputPower = MIN_POWER_FLOOR;
        } else if (outputPower < 0 && Math.abs(outputPower) < MIN_POWER_FLOOR) {
            outputPower = -MIN_POWER_FLOOR;
        }

        // 5. Max Power Clamp: Ensure power never exceeds the defined maximum.
        if (outputPower > MAX_POWER) outputPower = MAX_POWER;
        else if (outputPower < -MAX_POWER) outputPower = -MAX_POWER;

        // 6. Save state for the next loop iteration
        lastError = currentError;
        lastTime = currentTime;

        return outputPower;
    }
}