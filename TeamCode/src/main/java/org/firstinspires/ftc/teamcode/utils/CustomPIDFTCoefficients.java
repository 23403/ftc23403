package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.pedropathing.util.FeedForwardConstant;

public class CustomPIDFTCoefficients {
    public double P;
    public double I;
    public double D;
    public double F;
    public int TARGET;
    public FeedForwardConstant feedForwardConstantEquation;
    private boolean usingEquation;

    public CustomPIDFTCoefficients(double p, double i, double d, double f) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
        this.TARGET = 0;
    }

    public CustomPIDFTCoefficients(double p, double i, double d, FeedForwardConstant f) {
        this.usingEquation = true;
        this.P = p;
        this.I = i;
        this.D = d;
        this.feedForwardConstantEquation = f;
        this.TARGET = 0;
    }

    public CustomPIDFTCoefficients(double p, double i, double d, double f, int target) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
        this.TARGET = target;
    }

    public CustomPIDFTCoefficients(double p, double i, double d, FeedForwardConstant f, int target) {
        this.usingEquation = true;
        this.P = p;
        this.I = i;
        this.D = d;
        this.feedForwardConstantEquation = f;
        this.TARGET = target;
    }

    public double getCoefficient(double input) {
        return !this.usingEquation ? this.F : this.feedForwardConstantEquation.getConstant(input);
    }

    public void setCoefficients(double p, double i, double d, double f) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
    }

    @NonNull
    public String toString() {
        return "P: " + this.P + ", I: " + this.I + ", D: " + this.D + ", F: " + this.F;
    }
}