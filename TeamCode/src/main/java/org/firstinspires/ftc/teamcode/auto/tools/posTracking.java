package org.firstinspires.ftc.teamcode.auto.tools;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * MetroBotics/Code Conductors pos tracking with odometry.
 * Track the robot's position using telemetry for ease of points.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.2, 5/26/25
**/

@Config("Auto pos tracking")
@Autonomous(name = "Position tracking odometry", group = "tools_ftc23403")
public class posTracking extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    public static double startPosX = 9;
    public static double startPosY = 63.4;
    public static double startPosRotation = 0;
    private final Pose startPos = new Pose(startPosX, startPosY, Math.toRadians(startPosRotation)); // start Pos

    @Override
    public void init() {
        hardwareMap.get(IMU.class, ThreeWheelIMUConstants.IMU_HardwareMapName).resetYaw();
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(startPos);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void loop() {
        // telemetry for debugging
        telemetry.addData("x", poseUpdater.getPose().getX());
        telemetry.addData("y", poseUpdater.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(poseUpdater.getPose().getHeading()));
        telemetry.addData("total heading", Math.toDegrees(poseUpdater.getTotalHeading()));
        telemetry.update();
        // Draw the robot on the dashboard
        poseUpdater.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
