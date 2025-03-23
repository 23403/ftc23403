package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config("Limelight Configuration")
public class Limelight {
    private final Limelight3A limelight;
    private final Servo wristServo;
    private final Servo submersibleArm;
    private Telemetry telemetry = null;
    private LimelightState currentState = LimelightState.NO_SAMPLE_FOUND;
    public static LimelightConfig config = new LimelightConfig(
            0.5 / 27.0,
            0.05,
            1.0);

    public Limelight(Limelight3A limelight, Servo wristServo, Servo submersibleArm) {
        this.limelight = limelight;
        this.wristServo = wristServo;
        this.submersibleArm = submersibleArm;
    }

    public Limelight(Limelight3A limelight, Servo wristServo, Servo submersibleArm, Telemetry telemetry) {
        this.limelight = limelight;
        this.wristServo = wristServo;
        this.submersibleArm = submersibleArm;
        this.telemetry = telemetry;
    }

    public LimelightState getState() {
        return currentState;
    }

    public void updateTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public LimelightState search() {
        LLResult result = limelight.getLatestResult();

        switch (currentState) {
            case NO_SAMPLE_FOUND:
                if (result != null && result.isValid()) {
                    currentState = LimelightState.LOOKING_FOR_SAMPLE;
                }
                break;

            case LOOKING_FOR_SAMPLE:
                if (result.isValid()) {
                    double ta = result.getTa();
                    if (ta > 0.02) { // If we see a sample
                        currentState = LimelightState.MOVING_TO_SAMPLE;
                    }
                } else {
                    currentState = LimelightState.NO_SAMPLE_FOUND;
                }
                break;

            case MOVING_TO_SAMPLE:
                if (result.isValid()) {
                    double tx = result.getTx(); // Left/Right offset
                    double ta = result.getTa(); // Target area (size)

                    // ---- Wrist Rotation (tx to servo position) ----
                    double wristPosition = 0.5 + (tx * config.kRotationFactor);
                    wristPosition = Math.max(0, Math.min(1, wristPosition));
                    wristServo.setPosition(wristPosition);

                    // ---- Slide Extension (ta to servo position) ----
                    double slidePosition = 1.0 - ((ta - config.minTargetArea) / (config.maxTargetArea - config.minTargetArea)) * (1.0 - 0.45);
                    slidePosition = Math.max(0.45, Math.min(1.0, slidePosition));
                    submersibleArm.setPosition(slidePosition);

                    if (ta > 0.2) { // If close enough to grab
                        currentState = LimelightState.SAMPLE_REACHED;
                    }
                } else {
                    currentState = LimelightState.LOOKING_FOR_SAMPLE;
                }
                break;

            case SAMPLE_REACHED:
                // Keep wrist and slides in place, wait for grab command
                break;
        }

        // ---- Debugging Telemetry ----
        if (telemetry != null) {
            telemetry.addData("State", currentState);
            telemetry.addData("Valid Target", result != null && result.isValid());
            if (result != null && result.isValid()) {
                telemetry.addData("tx (Target X)", result.getTx());
                telemetry.addData("ta (Target Area)", result.getTa());
            }
            telemetry.addData("Wrist Servo", wristServo.getPosition());
            telemetry.addData("Slide Servo", submersibleArm.getPosition());
            telemetry.update();
        }

        // return
        return getState();
    }
}
