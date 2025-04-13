package org.firstinspires.ftc.teamcode.auto.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config("Auto pos tracking")
@Autonomous(name = "Position tracking odometry", group = ".test_ftc23403")
public class posTracking extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    public static double startPosX = 9;
    public static double startPosY = 63.4;
    public static double startPosRotation = 0;
    private final Pose startPos = new Pose(startPosX, startPosY, Math.toRadians(startPosRotation)); // start Pos

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        hardwareMap.get(IMU.class, ThreeWheelIMUConstants.IMU_HardwareMapName).resetYaw();
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(startPos);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void loop() {
        // Feedback to Driver Hub
        telemetry.addData("x", poseUpdater.getPose().getX());
        telemetry.addData("y", poseUpdater.getPose().getY());
        telemetry.addData("heading", poseUpdater.getPose().getHeading());
        telemetry.addData("total heading", poseUpdater.getTotalHeading());
        telemetry.update();
        // Draw the robot on the dashboard
        poseUpdater.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
