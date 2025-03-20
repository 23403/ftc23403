package org.firstinspires.ftc.teamcode.testCode;

/**
 * FeedforwardFun is a functional interface for computing feedforward.
 * It is essentially an alias for BiFunction<Double, Double?, Double>
 * that specifies that the first Double argument is position
 * and the second Double? argument is velocity.
 */
@FunctionalInterface
public interface FeedforwardFun {
    double compute(double position, Double velocity);
}
