package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import xyz.nin1275.utils.Timer;

@Config
public class Vision {
    public static class Limelight {
        private Limelight3A limelight;
        private LimelightState llState;
        private boolean validResult = false;
        private double tx = 0, ty = 0;
        public static double CAMERA_VERTICAL_CENTER_OFFSET = -14.5;
        public static double CAMERA_HORIZONTAL_CENTER_OFFSET = -5;
        // init
        public Limelight(Limelight3A limelight, LimelightState llState) {
            this.limelight = limelight;
            this.llState = llState;
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
            } else llState = LimelightState.NO_SAMPLE_FOUND;
        }
        // getters
        public double getRotation() {
            return MathUtils.clamp(tx, 0.0, 1.0);
        }
        public double getSubmersible() {
            return 1.0 - MathUtils.clamp(ty, 0.0, 1.0);
        }
        public LimelightState getState() {
            return llState;
        }
        // setter
        public void setState(LimelightState state) {
            llState = state;
        }
    }
}