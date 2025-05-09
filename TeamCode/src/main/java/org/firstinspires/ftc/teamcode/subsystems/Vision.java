package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import xyz.nin1275.utils.Timer;

@Config
public class Vision {
    public static class Limelight {
        private final Limelight3A limelight;
        private LimelightState llState;
        private Follower follower = null;
        boolean validResult = false;
        private double tx = -1, ty = -1;
        public static double CAMERA_VERTICAL_CENTER_OFFSET = -14.5;
        public static double CAMERA_HORIZONTAL_CENTER_OFFSET = -5;
        public static double STRAFE_SCALE = 20.0;
        // init
        public Limelight(Limelight3A limelight, LimelightState llState) {
            this.limelight = limelight;
            this.llState = llState;
            limelight.pipelineSwitch(1);
            limelight.start();
        }
        public Limelight(Limelight3A limelight, LimelightState llState, Follower follower) {
            this.limelight = limelight;
            this.llState = llState;
            this.follower = follower;
            limelight.pipelineSwitch(1);
            limelight.start();
        }
        // method
        public void update() {
            LLResult result = limelight.getLatestResult();
            validResult = (result != null && result.isValid());
            if (llState == LimelightState.FOUND_SAMPLE) {
                Timer.wait(200);
                setState(LimelightState.NO_SAMPLE_FOUND);
            } else if (validResult) {
                double distanceScale = 1.0 + (result.getTa() * 0.2);
                tx = ((result.getTx() - CAMERA_HORIZONTAL_CENTER_OFFSET + 27) / 54.0) * distanceScale;
                ty = (result.getTy() - CAMERA_VERTICAL_CENTER_OFFSET + 20) / 40.0;
                llState = LimelightState.FOUND_SAMPLE;
            } else {
                tx = -1;
                ty = -1;
                llState = LimelightState.NO_SAMPLE_FOUND;
            }
        }
        // getters
        public double getRotation() {
            return tx == -1 ? -1 : MathUtils.clamp(1.0 - tx, 0.0, 1.0);
        }
        public double getSubmersible() {
            return ty == -1 ? -1 : 1.0 - MathUtils.clamp(ty, 0.0, 1.0);
        }
        public LimelightState getState() {
            return llState;
        }
        public void strafe() {
            if (follower != null && getRotation() != -1) {
                double offset = (getRotation() - 0.5) * STRAFE_SCALE;
                Pose currentPose = follower.getPose();
                Pose targetPose = new Pose(
                        currentPose.getX(),              // No forward movement, just strafe
                        currentPose.getY() + offset,     // Strafe left/right based on sample position
                        currentPose.getHeading()
                );
                follower.followPath(new Path(new BezierLine(currentPose, targetPose)));
            }
        }
        // setters
        public void setState(LimelightState state) {
            this.llState = state;
        }
        public void setFollower(Follower follower) {
            this.follower = follower;
        }
    }
}