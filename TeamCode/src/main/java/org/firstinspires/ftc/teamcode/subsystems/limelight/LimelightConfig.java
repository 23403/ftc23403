package org.firstinspires.ftc.teamcode.subsystems.limelight;

import androidx.annotation.NonNull;

public class LimelightConfig {
    public double kRotationFactor = 0.5 / 27.0;
    public double minTargetArea = 0.05;  // Smallest target area (block far away)
    public double maxTargetArea = 1.0;   // Largest target area (block close)

    public LimelightConfig(double kRotationFactor, double minTargetArea, double maxTargetArea) {
        this.kRotationFactor = kRotationFactor;
        this.minTargetArea = minTargetArea;
        this.maxTargetArea = maxTargetArea;
    }

    @NonNull
    public String toString() {
        return "kRotationFactor: " + this.kRotationFactor + ", minTargetArea: " + this.minTargetArea + ", maxTargetArea: " + this.maxTargetArea;
    }
}