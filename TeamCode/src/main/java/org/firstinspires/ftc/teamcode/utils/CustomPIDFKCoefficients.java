package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

public class CustomPIDFKCoefficients {
    public double P;
    public double I;
    public double D;
    public double F;
    public double K;

    public CustomPIDFKCoefficients(double p, double i, double d, double f) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
        this.K = 0;
    }

    public CustomPIDFKCoefficients(double p, double i, double d) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = 0;
        this.K = 0;
    }

    public CustomPIDFKCoefficients(double p, double i, double d, double f, double K) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
        this.K = K;
    }

    public void setCoefficients(double p, double i, double d, double f) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.F = f;
    }

    @NonNull
    public String toString() {
        return "P: " + this.P + ", I: " + this.I + ", D: " + this.D + ", F: " + this.F + ", K: " + this.K;
    }
}