package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config("Limelight Python")
public class LimelightPy {
    private final Limelight3A limelight;
    private final Servo rotationServo;
    private final Servo subArmServo;
    private Telemetry telemetry;
    public static LimelightState currentState = LimelightState.NO_SAMPLE_FOUND;

    public LimelightPy(Limelight3A limelight, Servo rotationServo, Servo subArmServo, Telemetry telemetry) {
        this.limelight = limelight;
        this.rotationServo = rotationServo;
        this.subArmServo = subArmServo;
        this.telemetry = telemetry;
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public LimelightState getState() {
        return currentState;
    }

    public void updateTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            currentState = LimelightState.NO_SAMPLE_FOUND;
            return;
        }

        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs == null || pythonOutputs.length < 2) {
            currentState = LimelightState.NO_SAMPLE_FOUND;
            return;
        }

        double targetAngle = pythonOutputs[0]; // Rotation angle from SnapScript
        double targetDistance = pythonOutputs[1]; // Distance from SnapScript

        // Normalize rotation angle to servo range (0 to 1)
        double rotationPosition = Math.max(0, Math.min(1, 0.5 + targetAngle * (0.5 / 27.0)));
        rotationServo.setPosition(rotationPosition);

        // Normalize slide extension based on target distance
        double slidePosition = Math.max(0, Math.min(1, (targetDistance - 10.0) / (40.0 - 10.0)));
        subArmServo.setPosition(slidePosition);

        if (targetDistance < 12.0) {
            currentState = LimelightState.SAMPLE_REACHED;
        } else {
            currentState = LimelightState.MOVING_TO_SAMPLE;
        }

        if (telemetry != null) {
            telemetry.addData("State", currentState);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Target Distance", targetDistance);
            telemetry.addData("Rotation Servo Position", rotationPosition);
            telemetry.addData("Slide Servo Position", slidePosition);
            telemetry.update();
        }
    }
}
