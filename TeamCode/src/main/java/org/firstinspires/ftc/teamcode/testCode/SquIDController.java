package org.firstinspires.ftc.teamcode.testCode;

import static java.lang.Math.abs;
import static java.lang.Math.signum;
import static java.lang.Math.sqrt;

public class SquIDController extends PIDFController {
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
    public SquIDController(PIDCoefficients pid, double kV, double kA, double kStatic, FeedforwardFun kF) {
        super(pid, kV, kA, kStatic, kF);
    }

    public SquIDController(PIDCoefficients pid, FeedforwardFun kF) {
        this(pid, 0.0, 0.0, 0.0, kF);
    }

    @Override
    public double update(long timestamp, double measuredPosition, Double measuredVelocity) {
        double result = super.update(timestamp, measuredPosition, measuredVelocity);
        return sqrt(abs(result)) * signum(result);
    }
}
