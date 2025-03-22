package org.firstinspires.ftc.teamcode.testCode.PID_SquID;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.copySign;

/**
 * PID controller with various feedforward components.
 * Originally from Roadrunner 0.5
 * Ported to Kotlin by Zach.Waffle and j5155
 * Ported to Java by David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 */
public class PIDFController {
    /**
     * Feedforward parameters `kV`, `kA`, and `kStatic` correspond with a basic
     * kinematic model of DC motors. The general function `kF` computes a custom feedforward
     * term for other plants.
     *
     * @param pid     traditional PID coefficients
     * @param kV      feedforward velocity gain
     * @param kA      feedforward acceleration gain
     * @param kStatic additive feedforward constant
     * @param kF      custom feedforward that depends on position and velocity
     */
    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic, FeedforwardFun kF) {
        this.pid = pid;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
    }

    public PIDFController(PIDCoefficients pid, FeedforwardFun kF) {
        this(pid, 0.0, 0.0, 0.0, kF);
    }

    private PIDCoefficients pid;
    private double kV;
    private double kA;
    private double kStatic;
    private final FeedforwardFun kF;

    /**
     * Target position (that is, the controller setpoint).
     */
    public int targetPosition = 0;

    /**
     * Target velocity.
     */
    public double targetVelocity = 0.0;

    /**
     * Target acceleration.
     */
    public double targetAcceleration = 0.0;

    /**
     * Error computed in the last call to [.update]
     */
    public double lastError = 0.0;
    private double errorSum = 0.0;
    private long lastUpdateTs = 0;
    private boolean inputBounded = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;
    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;

    /**
     * Sets bound on the input of the controller. When computing the error, the min and max are
     * treated as the same value. (Imagine taking the segment of the real line between min and max
     * and attaching the endpoints.)
     *
     * @param min minimum input
     * @param max maximum input
     */
    public void setInputBounds(double min, double max) {
        assert min < max : "Min output must be less than max output!";
        inputBounded = true;
        minInput = min;
        maxInput = max;
    }

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    public void setOutputBounds(double min, double max) {
        assert min < max : "Min output must be less than max output!";
        outputBounded = true;
        minOutput = min;
        maxOutput = max;
    }

    public double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (abs(error) > inputRange / 2.0) {
                error -= copySign(inputRange, error);
            }
        }
        return error;
    }

    /**
     * Run a single iteration of the controller.
     *
     * @param timestamp        measurement timestamp as given by System.nanoTime
     * @param measuredPosition measured position (feedback)
     * @param measuredVelocity measured velocity
     */
    public double update(long timestamp, double measuredPosition, Double measuredVelocity) {
        double error = getPositionError(measuredPosition);

        if (lastUpdateTs == 0L) {
            lastError = error;
            lastUpdateTs = timestamp;
            return 0.0;
        }

        double dt = (timestamp - lastUpdateTs);
        errorSum += 0.5 * (error + lastError) * dt;
        double errorDeriv = (error - lastError) / dt;

        lastError = error;
        lastUpdateTs = timestamp;
        double velError = (measuredVelocity == null) ? errorDeriv : (targetVelocity - measuredVelocity);

        double baseOutput = pid.kP * error + pid.kI * errorSum + pid.kD * velError + kV * targetVelocity + kA * targetAcceleration
                + kF.compute(measuredPosition, measuredVelocity);

        double output = 0.0;
        if (abs(baseOutput) > 1e-6) {
            output = baseOutput + copySign(kStatic, baseOutput);
        }

        if (outputBounded) {
            return max(minOutput, min(output, maxOutput));
        }

        return output;
    }

    public double update(double measuredPosition) {
        return update(System.nanoTime(), measuredPosition, null);
    }

    /**
     * Reset the controller's integral sum.
     */
    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTs = 0;
    }

    // these must be var so that tuning with dash works
    public static class PIDCoefficients {
        public double kP;
        public double kI;
        public double kD;

        public PIDCoefficients(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
}
