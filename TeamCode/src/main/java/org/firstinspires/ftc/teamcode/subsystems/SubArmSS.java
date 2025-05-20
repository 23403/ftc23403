package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

public class SubArmSS {
    public static class TP {
        private final DcMotorEx subArm1;
        private final DcMotorEx subArm2;
        private final DcMotorEx subArm;
        private final boolean isDualMotor;
        private final ElapsedTime resetTimer = new ElapsedTime();
        private final double CPR;
        private final double INCHES_PER_REV;
        private double saLimitHigh;
        private double saLimitLow;
        private boolean saCorrection;
        private double K;
        private double F;
        private double slidesTARGET = 0;
        private SlidersStates sliderState = SlidersStates.FLOATING;
        private double inches1 = 0;
        private double inches2 = 0;
        private double inches = 0;
        private final PID controller;
        private int subArmCpos = 0;
        // init
        public TP(DcMotorEx motor1, DcMotorEx motor2, PID controller, double K, double F, double cpr, double inchesPerRev, double limitHigh, double limitLow, boolean correctionEnabled) {
            this.subArm1 = motor1;
            this.subArm2 = motor2;
            this.subArm = null;
            this.controller = controller;
            this.CPR = cpr;
            this.INCHES_PER_REV = inchesPerRev;
            this.saLimitHigh = limitHigh;
            this.saLimitLow = limitLow;
            this.saCorrection = correctionEnabled;
            this.K = K;
            this.F = F;
            this.isDualMotor = true;
            // init stuff
            resetTimer.reset();
            while (resetTimer.milliseconds() < 500) {
                subArm1.setPower(-1);
                subArm2.setPower(-1);
            }
            subArm1.setPower(0);
            subArm2.setPower(0);
            Motors.resetEncoders(subArm1, subArm2);
            Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm1, subArm2);
            resetTimer.reset();
        }
        public TP(DcMotorEx motor1, DcMotorEx motor2, PID controller, double K, double F, double cpr, double inchesPerRev) {
            this.subArm1 = motor1;
            this.subArm2 = motor2;
            this.subArm = null;
            this.controller = controller;
            this.CPR = cpr;
            this.INCHES_PER_REV = inchesPerRev;
            this.saLimitHigh = Double.MAX_VALUE;
            this.saLimitLow = Double.MIN_VALUE;
            this.saCorrection = true;
            this.K = K;
            this.F = F;
            this.isDualMotor = true;
            // init stuff
            resetTimer.reset();
            while (resetTimer.milliseconds() < 500) {
                subArm1.setPower(-1);
                subArm2.setPower(-1);
            }
            subArm1.setPower(0);
            subArm2.setPower(0);
            Motors.resetEncoders(subArm1, subArm2);
            Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm1, subArm2);
            resetTimer.reset();
        }
        public TP(DcMotorEx motor, PID controller, double F, double cpr, double inchesPerRev, double limitHigh, double limitLow, boolean correctionEnabled) {
            this.subArm1 = null;
            this.subArm2 = null;
            this.subArm = motor;
            this.controller = controller;
            this.CPR = cpr;
            this.INCHES_PER_REV = inchesPerRev;
            this.saLimitHigh = limitHigh;
            this.saLimitLow = limitLow;
            this.saCorrection = correctionEnabled;
            this.F = F;
            this.isDualMotor = false;
            // init stuff
            resetTimer.reset();
            while (resetTimer.milliseconds() < 500) {
                subArm.setPower(-1);
            }
            subArm.setPower(0);
            Motors.resetEncoders(subArm);
            Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm);
            resetTimer.reset();
        }
        public TP(DcMotorEx motor, PID controller, double F, double cpr, double inchesPerRev) {
            this.subArm1 = null;
            this.subArm2 = null;
            this.subArm = motor;
            this.controller = controller;
            this.CPR = cpr;
            this.INCHES_PER_REV = inchesPerRev;
            this.saLimitHigh = Double.MAX_VALUE;
            this.saLimitLow = Double.MIN_VALUE;
            this.saCorrection = true;
            this.F = F;
            this.isDualMotor = false;
            // init stuff
            resetTimer.reset();
            while (resetTimer.milliseconds() < 500) {
                subArm.setPower(-1);
            }
            subArm.setPower(0);
            Motors.resetEncoders(subArm);
            Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm);
            resetTimer.reset();
        }
        // loop
        public void update(boolean up, boolean down) {
            // vars
            double ff = saCorrection ? F : 0;
            // logic
            if (isDualMotor) {
                // get current positions
                int ticks1 = subArm1.getCurrentPosition();
                int ticks2 = subArm2.getCurrentPosition();
                // convert to inches
                inches1 = (ticks1 / CPR) * INCHES_PER_REV;
                inches2 = (ticks2 / CPR) * INCHES_PER_REV;
                // vars
                boolean movingUp = up && inches1 < saLimitHigh;
                boolean movingDown = down && inches1 > saLimitLow;
                // controls
                if (movingUp || movingDown) {
                    double target = movingUp ? saLimitHigh : saLimitLow;
                    double correction = (inches1 - inches2) * K;
                    subArm1.setTargetPosition((int) ((target / INCHES_PER_REV) * CPR));
                    subArm2.setTargetPosition((int) ((target / INCHES_PER_REV) * CPR));
                    subArm1.setPower(controller.getP());
                    subArm2.setPower(controller.getP() + correction);
                    subArmCpos = subArm1.getCurrentPosition();
                    Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm1, subArm2);
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else if (Math.abs(inches1 - saLimitLow) > 2 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    subArm1.setTargetPosition(subArmCpos);
                    subArm2.setTargetPosition(subArmCpos);
                    subArm1.setPower(ff);
                    subArm2.setPower(ff);
                    Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm1, subArm2);
                    if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
                    sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
                }
                // states
                if (Math.abs(inches1 - saLimitHigh) < 1 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    sliderState = SlidersStates.MAX_POS;
                } else if (Math.abs(inches1 - saLimitLow) < 2 &&
                        sliderState != SlidersStates.MOVING_TO_PRESET &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS &&
                        sliderState != SlidersStates.ZERO_POS_RESET &&
                        sliderState != SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(inches1 - saLimitLow) < 2) {
                        sliderState = SlidersStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }
                // reset slides 0 pos
                if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm1, subArm2);
                        subArm1.setPower(-0.1);
                        subArm2.setPower(-0.1);
                    } else {
                        subArm1.setPower(0);
                        subArm2.setPower(0);
                        Motors.resetEncoders(subArm1, subArm2);
                        Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm1, subArm2);
                        sliderState = SlidersStates.ZERO_POS_RESET;
                    }
                }
                // preset controls
                if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                    double correction = (inches1 - inches2) * K;
                    subArm1.setTargetPosition((int) ((slidesTARGET / INCHES_PER_REV) * CPR));
                    subArm2.setTargetPosition((int) ((slidesTARGET / INCHES_PER_REV) * CPR));
                    subArm1.setPower(controller.getP());
                    subArm2.setPower(controller.getP() + correction);
                    subArmCpos = subArm1.getCurrentPosition();
                    Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm1, subArm2);
                    if (Math.abs(inches1 - slidesTARGET) < 1) sliderState = SlidersStates.PRESET_REACHED;
                }
            } else {
                // get current positions
                int ticks = subArm.getCurrentPosition();
                // convert to inches
                inches = (ticks / CPR) * INCHES_PER_REV;
                // vars
                boolean movingUp = up && inches < saLimitHigh;
                boolean movingDown = down && inches > saLimitLow;
                // controls
                if (movingUp || movingDown) {
                    double target = movingUp ? saLimitHigh : saLimitLow;
                    subArm.setTargetPosition((int) ((target / INCHES_PER_REV) * CPR));
                    subArm.setPower(controller.getP());
                    subArmCpos = subArm.getCurrentPosition();
                    Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm);
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else if (Math.abs(inches - saLimitLow) > 2 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    subArm.setTargetPosition(subArmCpos);
                    subArm.setPower(ff);
                    Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm);
                    if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
                    sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
                }
                // states
                if (Math.abs(inches - saLimitHigh) < 1 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    sliderState = SlidersStates.MAX_POS;
                } else if (Math.abs(inches - saLimitLow) < 2 &&
                        sliderState != SlidersStates.MOVING_TO_PRESET &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS &&
                        sliderState != SlidersStates.ZERO_POS_RESET &&
                        sliderState != SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(inches - saLimitLow) < 2) {
                        sliderState = SlidersStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }
                // reset slides 0 pos
                if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm);
                        subArm.setPower(-0.1);
                    } else {
                        subArm.setPower(0);
                        Motors.resetEncoders(subArm);
                        Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm);
                        sliderState = SlidersStates.ZERO_POS_RESET;
                    }
                }
                // preset controls
                if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                    subArm.setTargetPosition((int) ((slidesTARGET / INCHES_PER_REV) * CPR));
                    subArm.setPower(controller.getP());
                    subArmCpos = subArm.getCurrentPosition();
                    Motors.setMode(DcMotorEx.RunMode.RUN_TO_POSITION, subArm);
                    if (Math.abs(inches - slidesTARGET) < 1) sliderState = SlidersStates.PRESET_REACHED;
                }
            }
        }
        // loop
        public void update() {
            // vars
            double ff = saCorrection ? F : 0;
            // logic
            if (isDualMotor) {
                // get current positions
                int ticks1 = subArm1.getCurrentPosition();
                int ticks2 = subArm2.getCurrentPosition();
                // convert to inches
                inches1 = (ticks1 / CPR) * INCHES_PER_REV;
                inches2 = (ticks2 / CPR) * INCHES_PER_REV;
                if (Math.abs(inches1 - saLimitLow) > 2 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    subArm1.setTargetPosition(subArmCpos);
                    subArm2.setTargetPosition(subArmCpos);
                    subArm1.setPower(ff);
                    subArm2.setPower(ff);
                    if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
                    sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
                }
                // states
                if (Math.abs(inches1 - saLimitHigh) < 1 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    sliderState = SlidersStates.MAX_POS;
                } else if (Math.abs(inches1 - saLimitLow) < 2 &&
                        sliderState != SlidersStates.MOVING_TO_PRESET &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS &&
                        sliderState != SlidersStates.ZERO_POS_RESET &&
                        sliderState != SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(inches1 - saLimitLow) < 2) {
                        sliderState = SlidersStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }
                // reset slides 0 pos
                if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        subArm1.setPower(-0.1);
                        subArm2.setPower(-0.1);
                    } else {
                        subArm1.setPower(0);
                        subArm2.setPower(0);
                        Motors.resetEncoders(subArm1, subArm2);
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm1, subArm2);
                        sliderState = SlidersStates.ZERO_POS_RESET;
                    }
                }
                // preset controls
                if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                    double correction = (inches1 - inches2) * K;
                    subArm1.setTargetPosition((int) ((slidesTARGET / INCHES_PER_REV) * CPR));
                    subArm2.setTargetPosition((int) ((slidesTARGET / INCHES_PER_REV) * CPR));
                    subArm1.setPower(controller.getP());
                    subArm2.setPower(controller.getP() + correction);
                    subArmCpos = subArm1.getCurrentPosition();
                    if (Math.abs(inches1 - slidesTARGET) < 1) sliderState = SlidersStates.PRESET_REACHED;
                }
            } else {
                // get current positions
                int ticks = subArm.getCurrentPosition();
                // convert to inches
                inches = (ticks / CPR) * INCHES_PER_REV;
                // controls
                if (Math.abs(inches - saLimitLow) > 2 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    subArm.setTargetPosition(subArmCpos);
                    subArm.setPower(ff);
                    if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
                    sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
                }
                // states
                if (Math.abs(inches - saLimitHigh) < 1 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    sliderState = SlidersStates.MAX_POS;
                } else if (Math.abs(inches - saLimitLow) < 2 &&
                        sliderState != SlidersStates.MOVING_TO_PRESET &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS &&
                        sliderState != SlidersStates.ZERO_POS_RESET &&
                        sliderState != SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(inches - saLimitLow) < 2) {
                        sliderState = SlidersStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }
                // reset slides 0 pos
                if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        subArm.setPower(-0.1);
                    } else {
                        subArm.setPower(0);
                        Motors.resetEncoders(subArm);
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm);
                        sliderState = SlidersStates.ZERO_POS_RESET;
                    }
                }
                // preset controls
                if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                    subArm.setTargetPosition((int) ((slidesTARGET / INCHES_PER_REV) * CPR));
                    subArm.setPower(controller.getP());
                    subArmCpos = subArm.getCurrentPosition();
                    if (Math.abs(inches - slidesTARGET) < 1) sliderState = SlidersStates.PRESET_REACHED;
                }
            }
        }
        // setters
        public void setSaCorrection(boolean saCorrection) {
            this.saCorrection = saCorrection;
        }
        public void setLimits(double highLimit, double lowLimit) {
            this.saLimitHigh = highLimit;
            this.saLimitLow = lowLimit;
        }
        public void moveTo(double target) {
            this.slidesTARGET = target;
            sliderState = SlidersStates.MOVING_TO_PRESET;
        }
        public void updatePIDKFValues(double p, double i, double d, double K, double F) {
            controller.setPID(p, i, d);
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
        public SlidersStates getState() {
            return sliderState;
        }
        public double getPower1() {
            return subArm1.getPower();
        }
        public double getPower2() {
            return subArm2.getPower();
        }
        public double getPower() {
            return subArm.getPower();
        }
        public ElapsedTime getResetTimer() {
            return resetTimer;
        }
    }

    public static class PD {
        private final DcMotorEx subArm1, subArm2, subArm;
        private final boolean isDualMotor;
        private final PID controller;
        private final ElapsedTime resetTimer = new ElapsedTime();
        private final double CPR;
        private final double INCHES_PER_REV;
        private double saLimitHigh;
        private double saLimitLow;
        private boolean saCorrection;
        private double K;
        private double F;
        private double slidesTARGET = 0;
        private SlidersStates sliderState = SlidersStates.FLOATING;
        private double inches1 = 0, inches2 = 0, inches = 0;

        public PD(DcMotorEx motor1, DcMotorEx motor2, PID controller, double K, double F, double cpr, double inchesPerRev, double limitHigh, double limitLow, boolean correctionEnabled) {
            this(motor1, motor2, controller, K, F, cpr, inchesPerRev);
            this.saLimitHigh = limitHigh;
            this.saLimitLow = limitLow;
            this.saCorrection = correctionEnabled;
        }

        public PD(DcMotorEx motor1, DcMotorEx motor2, PID controller, double K, double F, double cpr, double inchesPerRev) {
            this.subArm1 = motor1;
            this.subArm2 = motor2;
            this.subArm = null;
            this.controller = controller;
            this.CPR = cpr;
            this.INCHES_PER_REV = inchesPerRev;
            this.saLimitHigh = Double.MAX_VALUE;
            this.saLimitLow = Double.MIN_VALUE;
            this.saCorrection = true;
            this.K = K;
            this.F = F;
            this.isDualMotor = true;
            initializeMotors(subArm1, subArm2);
        }

        public PD(DcMotorEx motor, PID controller, double F, double cpr, double inchesPerRev, double limitHigh, double limitLow, boolean correctionEnabled) {
            this(motor, controller, F, cpr, inchesPerRev);
            this.saLimitHigh = limitHigh;
            this.saLimitLow = limitLow;
            this.saCorrection = correctionEnabled;
        }

        public PD(DcMotorEx motor, PID controller, double F, double cpr, double inchesPerRev) {
            this.subArm1 = null;
            this.subArm2 = null;
            this.subArm = motor;
            this.controller = controller;
            this.CPR = cpr;
            this.INCHES_PER_REV = inchesPerRev;
            this.saLimitHigh = Double.MAX_VALUE;
            this.saLimitLow = Double.MIN_VALUE;
            this.saCorrection = true;
            this.K = 0;
            this.F = F;
            this.isDualMotor = false;
            initializeMotors(subArm);
        }

        private void initializeMotors(DcMotorEx... motors) {
            resetTimer.reset();
            while (resetTimer.milliseconds() < 500) {
                for (DcMotorEx motor : motors) motor.setPower(-1);
            }
            for (DcMotorEx motor : motors) motor.setPower(0);
            Motors.resetEncoders(motors);
            Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
            resetTimer.reset();
        }

        public void update(boolean up, boolean down) {
            controller.setPID(Math.sqrt(controller.getP()), controller.getI(), controller.getD());
            double ff = calculateFeedforward();

            if (isDualMotor) {
                inches1 = encoderToInches(subArm1);
                inches2 = encoderToInches(subArm2);
                boolean movingUp = up && inches1 < saLimitHigh;
                boolean movingDown = down && inches1 > saLimitLow;

                if (movingUp || movingDown) {
                    double target = movingUp ? saLimitHigh : saLimitLow;
                    applyDualMotorControl(target, ff);
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else {
                    applyIdleDual(ff);
                    handleStateDual();
                }
            } else {
                inches = encoderToInches(subArm);
                boolean movingUp = up && inches < saLimitHigh;
                boolean movingDown = down && inches > saLimitLow;

                if (movingUp || movingDown) {
                    double target = movingUp ? saLimitHigh : saLimitLow;
                    applySingleMotorControl(target, ff);
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else {
                    applyIdleSingle(ff);
                    handleStateSingle();
                }
            }
        }

        public void update() {
            update(false, false);
        }

        private double encoderToInches(DcMotorEx motor) {
            return (motor.getCurrentPosition() / CPR) * INCHES_PER_REV;
        }

        private double clamp(double val) {
            return Math.max(-1, Math.min(1, val));
        }

        private double calculateFeedforward() {
            if (!saCorrection) return 0;
            double ratio = inches1 / saLimitHigh;
            if (ratio < 0.05) return -0.07;
            return F * Math.pow(ratio, 1.5);
        }

        private void applyDualMotorControl(double target, double ff) {
            double pid = controller.calculate(inches1, target);
            double rawPower = pid + ff;
            double correction = (inches1 - inches2) * K;
            subArm1.setPower(clamp(rawPower));
            subArm2.setPower(clamp(rawPower + correction));
        }

        private void applySingleMotorControl(double target, double ff) {
            double pid = controller.calculate(inches, target);
            subArm.setPower(clamp(pid + ff));
        }

        private void applyIdleDual(double ff) {
            subArm1.setPower(ff);
            subArm2.setPower(ff);
            if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
            sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
        }

        private void applyIdleSingle(double ff) {
            subArm.setPower(ff);
            if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
            sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
        }

        private void handleStateDual() {
            checkLimitsAndReset(inches1, subArm1, subArm2);
            if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                applyDualMotorControl(slidesTARGET, calculateFeedforward());
                if (Math.abs(inches1 - slidesTARGET) < 1) sliderState = SlidersStates.PRESET_REACHED;
            }
        }

        private void handleStateSingle() {
            checkLimitsAndReset(inches, subArm);
            if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                applySingleMotorControl(slidesTARGET, calculateFeedforward());
                if (Math.abs(inches - slidesTARGET) < 1) sliderState = SlidersStates.PRESET_REACHED;
            }
        }

        private void checkLimitsAndReset(double position, DcMotorEx... motors) {
            if (Math.abs(position - saLimitHigh) < 1 && sliderState != SlidersStates.MOVING_TO_PRESET) {
                sliderState = SlidersStates.MAX_POS;
            } else if (Math.abs(position - saLimitLow) < 2 && sliderState.ordinal() < SlidersStates.RESETTING_ZERO_POS.ordinal()) {
                sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                resetTimer.reset();
            }

            if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION && resetTimer.milliseconds() > 200 && Math.abs(position - saLimitLow) < 2) {
                sliderState = SlidersStates.RESETTING_ZERO_POS;
                resetTimer.reset();
            }

            if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                if (resetTimer.milliseconds() < 200) {
                    for (DcMotorEx motor : motors) motor.setPower(-0.1);
                } else {
                    for (DcMotorEx motor : motors) motor.setPower(0);
                    Motors.resetEncoders(motors);
                    Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, motors);
                    sliderState = SlidersStates.ZERO_POS_RESET;
                }
            }
        }
        // setters
        public void setSaCorrection(boolean saCorrection) {
            this.saCorrection = saCorrection;
        }
        public void setLimits(double highLimit, double lowLimit) {
            this.saLimitHigh = highLimit;
            this.saLimitLow = lowLimit;
        }
        public void moveTo(double target) {
            this.slidesTARGET = target;
            sliderState = SlidersStates.MOVING_TO_PRESET;
        }
        public void updatePIDKFValues(double p, double i, double d, double K, double F) {
            controller.setPID(p, i, d);
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
        public SlidersStates getState() {
            return sliderState;
        }
        public double getPower1() {
            return subArm1.getPower();
        }
        public double getPower2() {
            return subArm2.getPower();
        }
        public double getPower() {
            return subArm.getPower();
        }
        public ElapsedTime getResetTimer() {
            return resetTimer;
        }
    }
}
