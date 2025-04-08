package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides.F;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.variables.enums.extendArmStates;

import xyz.nin1275.utils.Timer;

@Config("ExtendArm Subsystem CONFIG")
public class ExtendArmSS {

    private final DcMotorEx extendArm1;
    private final DcMotorEx extendArm2;
    private final PIDController controller;

    // Constants
    public static double CPR = 384.16;
    public static double INCHES_PER_REV = 6.2832;

    // Target (in inches)
    private double targetInches;
    private double power;
    private boolean eaCorrection = true;

    // State Management
    private extendArmStates currentState;

    // Limits
    public static double eaLimitHigh = Double.MAX_VALUE;
    public static double eaLimitLow = Double.MAX_VALUE;

    // OFFSET
    public static double lowPosOFFSET = 1;
    public static double highPosOFFSET = 1;
    public static double presetOFFSET = 1;

    // Motion Profiling Variables
    private double lastPower1 = 0;
    private double lastPower2 = 0;
    public static double rampSpeed = 0.05;

    public ExtendArmSS(DcMotorEx extendArm1, DcMotorEx extendArm2) {
        this.extendArm1 = extendArm1;
        this.extendArm2 = extendArm2;

        // Init motor direction and encoders
        extendArm2.setDirection(DcMotorSimple.Direction.REVERSE);

        extendArm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        extendArm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Init PID controller
        controller = new PIDController(0, 0, 0);
        currentState = extendArmStates.FLOATING;
    }

    public void setLimits(double limitLow, double limitHigh) {
        eaLimitLow = limitLow;
        eaLimitHigh = limitHigh;
    }

    public void setEaCorrection(boolean correction) {
        this.eaCorrection = correction;
    }

    public void update() {
        controller.setPID(Math.sqrt(P), I, D);
        // Get current positions in inches
        double pos1 = ticksToInches(extendArm1.getCurrentPosition());
        double pos2 = ticksToInches(extendArm2.getCurrentPosition());

        // Determine Force Feedback based on position
        double ff = (eaCorrection && Math.abs(pos1 - eaLimitLow) > lowPosOFFSET) ? F : 0;

        // States handling
        switch (currentState) {
            case MANUAL_MOVEMENT:
                manualMovement(pos1, ff);
                break;

            case MOVING_TO_PRESET:
                moveToPreset(pos1, ff);
                break;

            case PRESET_REACHED:
                handlePresetReached(ff);
                break;

            case FORCE_FEED_BACK:
                forceFeedback(ff);
                break;
        }

        // Motion profiling and ramping for smoothness
        double rampedPower1 = smoothRamp(lastPower1, power, rampSpeed);
        double rampedPower2 = smoothRamp(lastPower2, power, rampSpeed);

        // Calculate sync error
        double syncError = pos1 - pos2;
        // Calculate correction power
        double correction = syncError * K;

        // Apply the power to the motors
        extendArm1.setPower(rampedPower1);
        extendArm2.setPower(rampedPower2 + correction);

        // Store last power for next frame
        lastPower1 = rampedPower1;
        lastPower2 = rampedPower2;
    }

    public void moveSlides(boolean up, boolean down) {
        // Handle movement based on button presses, respecting the limits
        if (up && ticksToInches(extendArm1.getCurrentPosition()) < eaLimitHigh) {
            currentState = extendArmStates.MOVING_TO_PRESET;
        } else if (down && ticksToInches(extendArm1.getCurrentPosition()) > eaLimitLow) {
            currentState = extendArmStates.MOVING_TO_PRESET;
        } else {
            currentState = eaCorrection ? extendArmStates.FORCE_FEED_BACK : extendArmStates.FLOATING;
        }
    }

    public void preset(double pos) {
        // Move slides to a preset target (in inches)
        targetInches = pos;
        currentState = extendArmStates.MOVING_TO_PRESET;
    }

    private void manualMovement(double pos1, double ff) {
        // Manual movement control based on dpad, respecting the limits
        if (pos1 < eaLimitHigh) {
            double pid = controller.calculate(pos1, eaLimitHigh);
            power = pid + ff;
            currentState = extendArmStates.MANUAL_MOVEMENT;
        } else if (pos1 > eaLimitLow) {
            double pid = controller.calculate(pos1, eaLimitLow);
            power = pid + ff;
            currentState = extendArmStates.MANUAL_MOVEMENT;
        }
    }

    private void moveToPreset(double pos1, double ff) {
        // Handle preset movement
        double pid = controller.calculate(pos1, targetInches);
        power = pid + ff;

        if (Math.abs(pos1 - targetInches) < presetOFFSET) {
            currentState = extendArmStates.PRESET_REACHED;
        }
    }

    private void handlePresetReached(double ff) {
        // Handling preset reached state
        forceFeedback(ff);
    }

    private void forceFeedback(double ff) {
        // Force feedback state (applies force feedback if required)
        extendArm1.setPower(ff);
        extendArm2.setPower(ff);
        if (currentState == extendArmStates.PRESET_REACHED) Timer.wait(500);
        currentState = eaCorrection ? extendArmStates.FORCE_FEED_BACK : extendArmStates.FLOATING;
    }

    private double ticksToInches(int ticks) {
        return (ticks / CPR) * INCHES_PER_REV;
    }

    private double smoothRamp(double lastPower, double targetPower, double speed) {
        if (targetPower > lastPower) {
            return Math.min(lastPower + speed, targetPower);
        } else if (targetPower < lastPower) {
            return Math.max(lastPower - speed, targetPower);
        }
        return targetPower;
    }

    public extendArmStates getCurrentState() {
        return currentState;
    }
}
