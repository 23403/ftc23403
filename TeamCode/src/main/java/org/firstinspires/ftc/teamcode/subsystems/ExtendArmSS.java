package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.variables.enums.extendArmStates;

import java.util.List;

import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

@Config("ExtendArm Subsystem CONFIG")
public class ExtendArmSS {
    // hardware
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private final PIDController controller;
    private final ElapsedTime resetTimer = new ElapsedTime();
    // constants
    public static double CPR = 384.16;
    public static double INCHES_PER_REV = 6.2832;
    private boolean eaCorrection = true;
    // limits
    public static double eaLimitHigh = Double.MAX_VALUE;
    public static double eaLimitLow = Double.MAX_VALUE;
    // OFFSETS
    public static double lowPosOFFSET = 2;
    public static double highPosOFFSET = 1;
    public static double presetOFFSET = 1;
    // motion profiling stuff
    private double lastPower = 0;
    public static double rampSpeed = 0.05;
    // stuff
    double pid;
    private double rawPower;
    private double syncError;
    double correction;
    private extendArmStates currentState;
    private double targetInches = 0;
    private boolean moveUp = false;

    // init
    public ExtendArmSS(DcMotorEx extendArm1, DcMotorEx extendArm2) {
        this.extendArm1 = extendArm1;
        this.extendArm2 = extendArm2;
        extendArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        Motors.resetEncoders(List.of(extendArm1, extendArm2));
        Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Init PID controller
        controller = new PIDController(0, 0, 0);
        currentState = extendArmStates.FLOATING;
    }
    // main
    public void update(boolean up, boolean down) {
        controller.setPID(Math.sqrt(P), I, D);
        // Get current positions in inches
        double pos1 = ticksToInches(extendArm1.getCurrentPosition());
        double pos2 = ticksToInches(extendArm2.getCurrentPosition());
        // Determine Force Feedback based on position
        double ff = (eaCorrection && Math.abs(pos1 - eaLimitLow) > lowPosOFFSET) ? F : 0;
        // controls
        if (up) {
            moveUp = true;
            currentState = extendArmStates.MANUAL_MOVEMENT;
        } else if (down) {
            moveUp = false;
            currentState = extendArmStates.MANUAL_MOVEMENT;
        } else if (Math.abs(pos1 - eaLimitLow) > 2 && currentState != extendArmStates.MOVING_TO_PRESET) {
            currentState = extendArmStates.FORCE_FEED_BACK;
        }
        // States handling
        switch (currentState) {
            case MANUAL_MOVEMENT:
                pid = controller.calculate(pos1, moveUp ? eaLimitHigh : eaLimitLow);
                rawPower = pid + ff;
                syncError = pos1 - pos2;
                correction = syncError * K;
                extendArm1.setPower(Math.max(-1, Math.min(1, motionProfilePower(rawPower)))); // leader
                extendArm2.setPower(Math.max(-1, Math.min(1, (motionProfilePower(rawPower) + correction)))); // follower with correction
                break;
            case MOVING_TO_PRESET:
                pid = controller.calculate(pos1, targetInches);
                rawPower = pid + ff;
                syncError = pos1 - pos2;
                correction = syncError * K;
                extendArm1.setPower(Math.max(-1, Math.min(1, motionProfilePower(rawPower)))); // leader
                extendArm2.setPower(Math.max(-1, Math.min(1, (motionProfilePower(rawPower) + correction)))); // follower with correction
                // check if we are at the target by an inch
                if (Math.abs(pos1 - targetInches) < presetOFFSET) {
                    currentState = extendArmStates.PRESET_REACHED;
                }
                break;
            case PRESET_REACHED:
                extendArm1.setPower(ff);
                extendArm2.setPower(ff);
                Timer.wait(500);
                currentState = eaCorrection ? extendArmStates.FORCE_FEED_BACK : extendArmStates.FLOATING;
                break;
            case FORCE_FEED_BACK:
                extendArm1.setPower(ff);
                extendArm2.setPower(ff);
                currentState = eaCorrection ? extendArmStates.FORCE_FEED_BACK : extendArmStates.FLOATING;
                break;
            case WAITING_FOR_RESET_CONFIRMATION:
                if (resetTimer.milliseconds() > 200 && Math.abs(pos1 - eaLimitLow) < lowPosOFFSET) {
                    resetTimer.reset();
                    currentState = extendArmStates.RESETTING_ZERO_POS;
                }
                break;
            case RESETTING_ZERO_POS:
                if (resetTimer.milliseconds() < 200) {
                    extendArm1.setPower(-0.4);
                    extendArm2.setPower(-0.4);
                } else {
                    extendArm1.setPower(0);
                    extendArm2.setPower(0);
                    Motors.resetEncoders(List.of(extendArm1, extendArm2));
                    Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    currentState = extendArmStates.LOW_POS;
                }
                break;
        }
        // checkers
        if (Math.abs(pos1 - eaLimitHigh) < highPosOFFSET && currentState != extendArmStates.MOVING_TO_PRESET) {
            currentState = extendArmStates.MAX_POS;
        } else if (Math.abs(pos1 - eaLimitLow) < lowPosOFFSET
                && currentState != extendArmStates.MOVING_TO_PRESET
                && currentState != extendArmStates.RESETTING_ZERO_POS
                && currentState != extendArmStates.LOW_POS
                && currentState != extendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            currentState = extendArmStates.WAITING_FOR_RESET_CONFIRMATION;
            resetTimer.reset();
        }
    }
    // setters
    public void setLimits(double limitLow, double limitHigh) {
        eaLimitLow = limitLow;
        eaLimitHigh = limitHigh;
    }
    public void setEaCorrection(boolean correction) {
        this.eaCorrection = correction;
    }
    public void preset(double targetInches) {
        this.targetInches = targetInches;
        currentState = extendArmStates.MOVING_TO_PRESET;
    }
    // getters
    public extendArmStates getCurrentState() {
        return currentState;
    }
    public double getTargetInches() {
        return targetInches;
    }
    public double getRawPower() {
        return rawPower;
    }
    public double getSyncError() {
        return syncError;
    }
    public ElapsedTime getResetTimer() {
        return resetTimer;
    }
    // utils
    public double ticksToInches(int ticks) {
        return (ticks / CPR) * INCHES_PER_REV;
    }
    private double motionProfilePower(double power) {
        lastPower = smoothRamp(lastPower, power, rampSpeed);
        return smoothRamp(lastPower, rawPower, rampSpeed);
    }
    private double smoothRamp(double lastPower, double targetPower, double speed) {
        if (targetPower > lastPower) {
            return Math.min(lastPower + speed, targetPower);
        } else if (targetPower < lastPower) {
            return Math.max(lastPower - speed, targetPower);
        }
        return targetPower;
    }
    public Command resetSlidesINIT = new SequentialCommandGroup(
            new InstantCommand(() -> {
                resetTimer.reset();
                extendArm1.setPower(-0.3);
                extendArm2.setPower(-0.3);
            }),
            new WaitCommand(500),
            new InstantCommand(() -> {
                extendArm1.setPower(0);
                extendArm2.setPower(0);
                Motors.resetEncoders(List.of(extendArm1, extendArm2));
                Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                resetTimer.reset();
            })
    );
    public void resetSlides() {
        if (currentState != extendArmStates.RESETTING_ZERO_POS
                && currentState != extendArmStates.LOW_POS
                && currentState != extendArmStates.WAITING_FOR_RESET_CONFIRMATION
                && currentState != extendArmStates.MOVING_TO_PRESET
                && currentState != extendArmStates.MAX_POS && currentState != extendArmStates.MANUAL_MOVEMENT) {
            currentState = extendArmStates.WAITING_FOR_RESET_CONFIRMATION;
            resetTimer.reset();
        }
    }
}
