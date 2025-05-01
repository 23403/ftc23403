package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Config
public class Vision {
    // Limelight and claw configuration
    public static double limelightHeight = 9.5; // Camera height in inches
    public static double limelightAngle = 60; // Camera angle (0° = down, 90° = forward)
    public static double clawForwardOffset = 19; // Claw's forward offset from the camera
    public static double clawLateralOffset = 5; // Claw's lateral (right is +) offset from the camera

    private Pose target = new Pose(); // The best sample's position
    private Pose cachedTarget; // Cached best sample
    private final Limelight3A limelight;
    private PathChain toTarget;
    private final Telemetry telemetry;
    private final int[] unwanted;
    private double bestAngle;
    private final Follower f;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, int[] unwanted, Follower f) {
        this.unwanted = unwanted;
        this.telemetry = telemetry;
        this.f = f;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(9);
        limelight.start();
        f.update();
        cachedTarget = f.getPose();
        f.update();
    }

    public void find() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        if (detections.isEmpty()) {
            telemetry.addData("Detections", "None");
            target = cachedTarget.copy();
            return;
        }

        // List to store scored detections
        List<LL3ADetection> scoredDetections = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : detections) {
            int c = detection.getClassId();

            boolean colorMatch = true;

            for (int o : unwanted) {
                if (c == o) {
                    colorMatch = false;
                    break;
                }
            }

            if (colorMatch) {
                // Compute angles
                double actualYAngle = Math.toRadians(limelightAngle + detection.getTargetYDegrees());
                double xAngle = Math.toRadians(-detection.getTargetXDegrees());

                // Compute distances
                double yDistance = limelightHeight / Math.tan(actualYAngle); // Forward distance (FTC X)
                double xDistance = Math.tan(xAngle) * yDistance; // Lateral distance (FTC Y)

                // Score based on alignment
                double rotationScore = -Math.abs((detection.getTargetCorners().get(0).get(0) -
                        detection.getTargetCorners().get(1).get(0)) /
                        (detection.getTargetCorners().get(1).get(1) -
                                detection.getTargetCorners().get(2).get(1)) - (1.5 / 3.5));
                double score = -yDistance - Math.abs(xDistance) + 2.0 * rotationScore; // Weighted scoring

                double angle;

                if (detection.getTargetCorners() == null || detection.getTargetCorners().size() < 4) {
                    angle = Double.NaN;
                }

                List<List<Double>> corners = detection.getTargetCorners();

                double dx = Math.toRadians(corners.get(1).get(0) - corners.get(0).get(0));
                double dy = Math.toRadians(corners.get(2).get(1) - corners.get(0).get(1));
                angle = ((Math.atan2(dy, dx)) * 4.5);

                scoredDetections.add(new LL3ADetection(detection, score, yDistance, xDistance, rotationScore, angle));
            }
        }

        // Find the best detection

        if (!scoredDetections.isEmpty()) {
            scoredDetections.sort(Comparator.comparingDouble(LL3ADetection::getScore).reversed());
            LL3ADetection bestDetection = scoredDetections.get(0);

            bestAngle = bestDetection.getAngle();

            // Convert to coordinates and apply claw offsets
            // X (forward)
            // Y (left)
            Pose sample = new Pose(
                    bestDetection.getYDistance(), // X (forward)
                    bestDetection.getXDistance(), // Y (left)
                    0
            );

            Pose difference = new Pose(sample.getX() - clawForwardOffset + 2.5, sample.getY() + clawLateralOffset, 0);

            target = new Pose(f.getPose().getX() + difference.getX(), f.getPose().getY() + difference.getY(), f.getPose().getHeading());
            cachedTarget = target.copy();

            toTarget = new PathBuilder()
                    .addPath(new BezierLine(f.getPose(), target)).setConstantHeadingInterpolation(f.getPose().getHeading()).build();

            // Display results
            telemetry.addData("Best Detection", bestDetection.getDetection().getClassName());
            telemetry.addData("Sample Position", "X: %.2f, Y: %.2f", sample.getX(), sample.getY());
            telemetry.addData("diff", difference);
            telemetry.addData("target", target);
            telemetry.addData("current", f.getPose());
        } else {
            target = cachedTarget.copy();
        }
    }

    public Pose getTarget() {
        return target;
    }

    public PathChain toTarget() {
        return toTarget;
    }

    public void off() {
        limelight.stop();
    }

    public void on() {
        limelight.start();
    }

    public double getAngle() {
        return bestAngle;
    }
}