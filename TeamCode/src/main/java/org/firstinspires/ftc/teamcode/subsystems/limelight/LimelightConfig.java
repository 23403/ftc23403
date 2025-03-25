package org.firstinspires.ftc.teamcode.subsystems.limelight;

import androidx.annotation.NonNull;

public class LimelightConfig {
    public double kRotationFactor = 0.5 / 27.0;
    public double minTargetArea = 0.05;  // Smallest target area (block far away)
    public double maxTargetArea = 1.0;   // Largest target area (block close)
    // Constants for distance calculation
    public double LIMELIGHT_MOUNT_ANGLE_DEGREES = 25.0; // Adjust based on your mount
    public double LIMELIGHT_LENS_HEIGHT_INCHES = 20.0;  // Adjust based on robot
    public double TARGET_HEIGHT_INCHES = 60.0;          // Adjust based on field target

    public LimelightConfig(double kRotationFactor, double minTargetArea, double maxTargetArea, double LIMELIGHT_MOUNT_ANGLE_DEGREES, double LIMELIGHT_LENS_HEIGHT_INCHES, double TARGET_HEIGHT_INCHES) {
        this.kRotationFactor = kRotationFactor;
        this.minTargetArea = minTargetArea;
        this.maxTargetArea = maxTargetArea;
        this.LIMELIGHT_MOUNT_ANGLE_DEGREES = LIMELIGHT_MOUNT_ANGLE_DEGREES;
        this.LIMELIGHT_LENS_HEIGHT_INCHES = LIMELIGHT_LENS_HEIGHT_INCHES;
        this.TARGET_HEIGHT_INCHES = TARGET_HEIGHT_INCHES;
    }

    @NonNull
    public String toString() {
        return "kRotationFactor: " + this.kRotationFactor + ", minTargetArea: " + this.minTargetArea + ", maxTargetArea: " + this.maxTargetArea;
    }
}