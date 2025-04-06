package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Date;

@Config("Limelight Configuration")
public class LimelightPy {
    private Limelight3A limelight;
    private final Servo rotationServo;
    private final Servo subArmServo;
    private Telemetry telemetry = null;
    private LimelightState currentState = LimelightState.NO_SAMPLE_FOUND;

    public static double minDistance = 10.0;
    public static double maxDistance = 40.0;

    public static double kRotationFactor = 27.0; // Scale factor for rotation

    public LimelightPy(Limelight3A limelight, Servo rotationServo, Servo subArmServo) {
        this.limelight = limelight;
        this.rotationServo = rotationServo;
        this.subArmServo = subArmServo;
    }

    public LimelightPy(Limelight3A limelight, Servo rotationServo, Servo subArmServo, Telemetry telemetry) {
        this(limelight, rotationServo, subArmServo);
        this.telemetry = telemetry;
    }

    public LimelightState getState() {
        return currentState;
    }

    public void updateTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void updateLimelight(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void update() {
        if (telemetry != null) {
            telemetry.addLine("Running Limelight update...");
        }

        limelight.pipelineSwitch(0);

        LLResult result = limelight.getLatestResult();

        if (telemetry != null) {
            telemetry.addData("LL Result", result != null ? "Got result" : "NULL");
            telemetry.addData("LL Valid", result != null && result.isValid());
        }

        if (result == null || !result.isValid() || result.getPythonOutput() == null || result.getPythonOutput().length < 5) {
            currentState = LimelightState.NO_SAMPLE_FOUND;
            if (telemetry != null) telemetry.update();
            return;
        }

        double[] pythonOutputs = result.getPythonOutput();

        double distance = pythonOutputs[3];
        double angle = pythonOutputs[4];

        // Clamp and scale angle to servo position (servo range: 0.43 to 0.55)
        angle = Math.max(-27, Math.min(27, angle));
        double wristPosition = 0.49 + (angle / kRotationFactor) * 0.06;
        wristPosition = Math.max(0.43, Math.min(0.55, wristPosition));

        // Clamp and scale distance to servo position
        double slidePosition = 1.0 - ((distance - minDistance) / (maxDistance - minDistance)) * (1.0 - 0.45);
        slidePosition = Math.max(0.45, Math.min(1.0, slidePosition));

        rotationServo.setPosition(wristPosition);
        subArmServo.setPosition(slidePosition);

        currentState = (distance < 12.0) ? LimelightState.SAMPLE_REACHED : LimelightState.MOVING_TO_SAMPLE;

        if (telemetry != null) {
            telemetry.addData("State", currentState);
            telemetry.addData("Angle (Python)", angle);
            telemetry.addData("Distance (Python)", distance);
            telemetry.addData("Rotation Servo", wristPosition);
            telemetry.addData("SubArm Servo", slidePosition);
            telemetry.addData("Raw LL Python", Arrays.toString(pythonOutputs));
            telemetry.addData("LL Time since last update", new Date(limelight.getTimeSinceLastUpdate()));
            telemetry.addData("pythonOutputs", Arrays.toString(pythonOutputs));
            telemetry.update();
        }
    }
}