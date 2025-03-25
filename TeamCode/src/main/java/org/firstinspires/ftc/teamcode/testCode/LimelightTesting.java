package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightState;

@Autonomous(name="Limelight Testing", group="test_ftc23403")
public class LimelightTesting extends OpMode {
    private Limelight3A limelight;
    private Servo wristServo;
    private Servo submersibleArm;
    private LimelightState currentState = LimelightState.NO_SAMPLE_FOUND; // Start state
    /**
     * Initialization code.
     */
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        wristServo = hardwareMap.get(Servo.class, "wrist2");
        submersibleArm = hardwareMap.get(Servo.class, "subArm");
        // combine both FTCDashboard and the regular telemetry
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        // limelight stuff
        limelight.setPollRateHz(100); // Ask for data 100 times per second
        limelight.start(); // Start tracking
        // telemetry
        telemetry.addLine("Use this to config limelight.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     */
    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        switch (currentState) {
            case NO_SAMPLE_FOUND:
                if (result != null && result.isValid()) {
                    currentState = LimelightState.LOOKING_FOR_SAMPLE;
                }
                break;

            case LOOKING_FOR_SAMPLE:
                if (result.isValid()) {
                    double ty = result.getTy();
                    if (ty > -20) { // If we see a sample
                        currentState = LimelightState.MOVING_TO_SAMPLE;
                    }
                } else {
                    currentState = LimelightState.NO_SAMPLE_FOUND;
                }
                break;

            case MOVING_TO_SAMPLE:
                if (result.isValid()) {
                    double tx = result.getTx(); // Left/Right offset
                    double ty = result.getTy(); // Vertical angle

                    // ---- Calculate Distance ----
                    double angleToGoalDegrees = Limelight.config.LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
                    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
                    double distanceFromLimelightToGoalInches =
                            (Limelight.config.TARGET_HEIGHT_INCHES - Limelight.config.LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);

                    // ---- Wrist Rotation (tx to servo position) ----
                    double kRotationFactor = 0.5 / 27.0;
                    double wristPosition = 0.5 + (tx * kRotationFactor);
                    wristPosition = Math.max(0, Math.min(1, wristPosition));
                    wristServo.setPosition(wristPosition);

                    // ---- Slide Extension (using distance) ----
                    double minDistance = 10.0;  // Closest distance
                    double maxDistance = 40.0;  // Farthest distance
                    double slidePosition = 1.0 - ((distanceFromLimelightToGoalInches - minDistance) / (maxDistance - minDistance)) * (1.0 - 0.45);
                    slidePosition = Math.max(0.45, Math.min(1.0, slidePosition));
                    submersibleArm.setPosition(slidePosition);

                    if (distanceFromLimelightToGoalInches < 12.0) { // Close enough to grab
                        currentState = LimelightState.SAMPLE_REACHED;
                    }
                } else {
                    currentState = LimelightState.LOOKING_FOR_SAMPLE;
                }
                break;

            case SAMPLE_REACHED:
                telemetry.addData("Status", "Sample Reached!");
                break;
        }

        // ---- Debugging Telemetry ----
        telemetry.addData("State", currentState);
        telemetry.addData("Valid Target", result != null && result.isValid());
        if (result != null && result.isValid()) {
            telemetry.addData("tx (Target X)", result.getTx());
            telemetry.addData("ty (Target Y)", result.getTy());
            telemetry.addData("Distance (inches)", (Limelight.config.TARGET_HEIGHT_INCHES - Limelight.config.LIMELIGHT_LENS_HEIGHT_INCHES) / Math.tan(Math.toRadians(Limelight.config.LIMELIGHT_MOUNT_ANGLE_DEGREES + result.getTy())));
        }
        telemetry.addData("Wrist Servo", wristServo.getPosition());
        telemetry.addData("Slide Servo", submersibleArm.getPosition());
        telemetry.update();
    }
}