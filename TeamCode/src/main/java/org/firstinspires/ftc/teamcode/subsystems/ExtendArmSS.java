package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;

import java.util.List;

import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

public class ExtendArmSS {
    private final DcMotorEx extendArm1, extendArm2;
    private final DcMotorEx extendArm;
    private final boolean isDualMotor;
    private final PIDController controller;
    private final ElapsedTime resetTimer = new ElapsedTime();
    private final double CPR;
    private final double INCHES_PER_REV;
    private double eaLimitHigh;
    private double eaLimitLow;
    private boolean eaCorrection;
    private double K;
    private double F;
    private double slidesTARGET = 0;
    private ExtendArmStates extendArmState = ExtendArmStates.FLOATING;
    private double inches1 = 0;
    private double inches2 = 0;
    private double inches = 0;

    // init
    public ExtendArmSS(DcMotorEx motor1, DcMotorEx motor2, PIDController controller, double K, double F, double cpr, double inchesPerRev, double limitHigh, double limitLow, boolean correctionEnabled) {
        this.extendArm1 = motor1;
        this.extendArm2 = motor2;
        this.extendArm = null;
        this.controller = controller;
        this.CPR = cpr;
        this.INCHES_PER_REV = inchesPerRev;
        this.eaLimitHigh = limitHigh;
        this.eaLimitLow = limitLow;
        this.eaCorrection = correctionEnabled;
        this.K = K;
        this.F = F;
        this.isDualMotor = true;
        // init stuff
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            extendArm1.setPower(-0.4);
            extendArm2.setPower(-0.4);
        }
        extendArm1.setPower(0);
        extendArm2.setPower(0);
        Motors.resetEncoders(extendArm1, extendArm2);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
        resetTimer.reset();
    }
    public ExtendArmSS(DcMotorEx motor1, DcMotorEx motor2, PIDController controller, double K, double F, double cpr, double inchesPerRev) {
        this.extendArm1 = motor1;
        this.extendArm2 = motor2;
        this.extendArm = null;
        this.controller = controller;
        this.CPR = cpr;
        this.INCHES_PER_REV = inchesPerRev;
        this.eaLimitHigh = Double.MAX_VALUE;
        this.eaLimitLow = Double.MIN_VALUE;
        this.eaCorrection = true;
        this.K = K;
        this.F = F;
        this.isDualMotor = true;
        // init stuff
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            extendArm1.setPower(-0.4);
            extendArm2.setPower(-0.4);
        }
        extendArm1.setPower(0);
        extendArm2.setPower(0);
        Motors.resetEncoders(extendArm1, extendArm2);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
        resetTimer.reset();
    }
    public ExtendArmSS(DcMotorEx motor, PIDController controller, double K, double F, double cpr, double inchesPerRev, double limitHigh, double limitLow, boolean correctionEnabled) {
        this.extendArm1 = null;
        this.extendArm2 = null;
        this.extendArm = motor;
        this.controller = controller;
        this.CPR = cpr;
        this.INCHES_PER_REV = inchesPerRev;
        this.eaLimitHigh = limitHigh;
        this.eaLimitLow = limitLow;
        this.eaCorrection = correctionEnabled;
        this.K = K;
        this.F = F;
        this.isDualMotor = false;
        // init stuff
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            extendArm.setPower(-0.4);
        }
        extendArm.setPower(0);
        Motors.resetEncoders(extendArm);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm);
        resetTimer.reset();
    }
    public ExtendArmSS(DcMotorEx motor, PIDController controller, double K, double F, double cpr, double inchesPerRev) {
        this.extendArm1 = null;
        this.extendArm2 = null;
        this.extendArm = motor;
        this.controller = controller;
        this.CPR = cpr;
        this.INCHES_PER_REV = inchesPerRev;
        this.eaLimitHigh = Double.MAX_VALUE;
        this.eaLimitLow = Double.MIN_VALUE;
        this.eaCorrection = true;
        this.K = K;
        this.F = F;
        this.isDualMotor = false;
        // init stuff
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            extendArm.setPower(-0.4);
        }
        extendArm.setPower(0);
        Motors.resetEncoders(extendArm);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm);
        resetTimer.reset();
    }
    // loop
    public void update(Gamepad gamepad) {
        controller.setPID(Math.sqrt(controller.getP()), controller.getI(), controller.getD());
        // vars
        double ff = eaCorrection ? F : 0;
        // logic
        if (isDualMotor) {
            // get current positions
            int ticks1 = extendArm1.getCurrentPosition();
            int ticks2 = extendArm2.getCurrentPosition();
            // convert to inches
            inches1 = (ticks1 / CPR) * INCHES_PER_REV;
            inches2 = (ticks2 / CPR) * INCHES_PER_REV;
            // vars
            boolean movingUp = gamepad.dpad_up && inches1 < eaLimitHigh;
            boolean movingDown = gamepad.dpad_down && inches1 > eaLimitLow;
            // controls
            if (movingUp || movingDown) {
                double target = movingUp ? eaLimitHigh : eaLimitLow;
                double pid = controller.calculate(inches1, target);
                double rawPower = pid + ff;
                double correction = (inches1 - inches2) * K;
                extendArm1.setPower(clamp(rawPower));
                extendArm2.setPower(clamp(rawPower + correction));
                extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
            } else if (Math.abs(inches1 - eaLimitLow) > 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                extendArm1.setPower(ff);
                extendArm2.setPower(ff);
                if (extendArmState == ExtendArmStates.PRESET_REACHED) Timer.wait(500);
                extendArmState = eaCorrection ? ExtendArmStates.FORCE_FEED_BACK : ExtendArmStates.FLOATING;
            }
            // states
            if (Math.abs(inches1 - eaLimitHigh) < 1 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                extendArmState = ExtendArmStates.MAX_POS;
            } else if (Math.abs(inches1 - eaLimitLow) < 2 &&
                    extendArmState != ExtendArmStates.MOVING_TO_PRESET &&
                    extendArmState != ExtendArmStates.RESETTING_ZERO_POS &&
                    extendArmState != ExtendArmStates.ZERO_POS_RESET &&
                    extendArmState != ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                extendArmState = ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION;
                resetTimer.reset();
            }
            // pre resetting slides pos
            if (extendArmState == ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                if (resetTimer.milliseconds() > 200 && Math.abs(inches1 - eaLimitLow) < 2) {
                    extendArmState = ExtendArmStates.RESETTING_ZERO_POS;
                    resetTimer.reset();
                }
            }
            // reset slides 0 pos
            if (extendArmState == ExtendArmStates.RESETTING_ZERO_POS) {
                if (resetTimer.milliseconds() < 200) {
                    extendArm1.setPower(-0.1);
                    extendArm2.setPower(-0.1);
                } else {
                    extendArm1.setPower(0);
                    extendArm2.setPower(0);
                    Motors.resetEncoders(extendArm1, extendArm2);
                    Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
                    extendArmState = ExtendArmStates.ZERO_POS_RESET;
                }
            }
            // preset controls
            if (extendArmState == ExtendArmStates.MOVING_TO_PRESET) {
                double pid = controller.calculate(inches1, slidesTARGET);
                double rawPower = pid + ff;
                double correction = (inches1 - inches2) * K;
                extendArm1.setPower(clamp(rawPower));
                extendArm2.setPower(clamp(rawPower + correction));
                if (Math.abs(inches1 - slidesTARGET) < 1) extendArmState = ExtendArmStates.PRESET_REACHED;
            }
        } else {
            // get current positions
            int ticks = extendArm.getCurrentPosition();
            // convert to inches
            inches = (ticks / CPR) * INCHES_PER_REV;
            // vars
            boolean movingUp = gamepad.dpad_up && inches < eaLimitHigh;
            boolean movingDown = gamepad.dpad_down && inches > eaLimitLow;
            // controls
            if (movingUp || movingDown) {
                double target = movingUp ? eaLimitHigh : eaLimitLow;
                double pid = controller.calculate(inches, target);
                double rawPower = pid + ff;
                extendArm.setPower(clamp(rawPower));
                extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
            } else if (Math.abs(inches - eaLimitLow) > 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                extendArm.setPower(ff);
                if (extendArmState == ExtendArmStates.PRESET_REACHED) Timer.wait(500);
                extendArmState = eaCorrection ? ExtendArmStates.FORCE_FEED_BACK : ExtendArmStates.FLOATING;
            }
            // states
            if (Math.abs(inches - eaLimitHigh) < 1 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                extendArmState = ExtendArmStates.MAX_POS;
            } else if (Math.abs(inches - eaLimitLow) < 2 &&
                    extendArmState != ExtendArmStates.MOVING_TO_PRESET &&
                    extendArmState != ExtendArmStates.RESETTING_ZERO_POS &&
                    extendArmState != ExtendArmStates.ZERO_POS_RESET &&
                    extendArmState != ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                extendArmState = ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION;
                resetTimer.reset();
            }
            // pre resetting slides pos
            if (extendArmState == ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                if (resetTimer.milliseconds() > 200 && Math.abs(inches - eaLimitLow) < 2) {
                    extendArmState = ExtendArmStates.RESETTING_ZERO_POS;
                    resetTimer.reset();
                }
            }
            // reset slides 0 pos
            if (extendArmState == ExtendArmStates.RESETTING_ZERO_POS) {
                if (resetTimer.milliseconds() < 200) {
                    extendArm.setPower(-0.1);
                } else {
                    extendArm.setPower(0);
                    Motors.resetEncoders(extendArm);
                    Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm);
                    extendArmState = ExtendArmStates.ZERO_POS_RESET;
                }
            }
            // preset controls
            if (extendArmState == ExtendArmStates.MOVING_TO_PRESET) {
                double pid = controller.calculate(inches, slidesTARGET);
                double rawPower = pid + ff;
                extendArm.setPower(clamp(rawPower));
                if (Math.abs(inches - slidesTARGET) < 1) extendArmState = ExtendArmStates.PRESET_REACHED;
            }
        }
    }
    // setters
    public void setEaCorrection(boolean eaCorrection) {
        this.eaCorrection = eaCorrection;
    }
    public void setLimits(double highLimit, double lowLimit) {
        this.eaLimitHigh = highLimit;
        this.eaLimitLow = lowLimit;
    }
    public void moveTo(double target) {
        this.slidesTARGET = target;
        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
    }
    public void updatePIDKFValues(double p, double i, double d, double K, double F) {
        controller.setPID(Math.sqrt(p), i, d);
        this.K = K;
        this.F = F;
    }
    // debugging values
    public double getInches1() {
        return inches1;
    }
    public double getInches2() {
        return inches2;
    }
    public double getInches() {
        return inches;
    }
    public double getTarget() {
        return slidesTARGET;
    }
    public ExtendArmStates getState() {
        return extendArmState;
    }
    public double getPower1() {
        return extendArm1.getPower();
    }
    public double getPower2() {
        return extendArm2.getPower();
    }
    public double getPower() {
        return extendArm.getPower();
    }
    public ElapsedTime getResetTimer() {
        return resetTimer;
    }
    // utils
    private double clamp(double val) {
        return Math.max(-1, Math.min(1, val));
    }
}
