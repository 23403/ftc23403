package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */

@Config
@Autonomous (name = "Auto Hook Odometry", group = ".ftc23403")
public class AutoHook extends OpMode {
    private Telemetry telemetryA;

    public static double RADIUS = 10;

    private Follower follower;

    private PathChain specimen;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        specimen = follower.pathBuilder()
                .addPath(
                // Line 1
                new BezierCurve(
                        new Point(9.757, 84.983, Point.CARTESIAN),
                        new Point(37.352, 115.495, Point.CARTESIAN),
                        new Point(36.668, 84.983, Point.CARTESIAN)
                )
        )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(36.668, 84.983, Point.CARTESIAN),
                                new Point(94.000, 64.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(94.000, 64.000, Point.CARTESIAN),
                                new Point(35.000, 98.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(35.000, 98.000, Point.CARTESIAN),
                                new Point(142.000, 118.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(142.000, 118.000, Point.CARTESIAN),
                                new Point(7.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(7.000, 100.000, Point.CARTESIAN),
                                new Point(108.000, 7.000, Point.CARTESIAN),
                                new Point(113.000, 104.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(113.000, 104.000, Point.CARTESIAN),
                                new Point(17.000, 140.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(17.000, 140.000, Point.CARTESIAN),
                                new Point(86.000, 113.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(86.000, 113.000, Point.CARTESIAN),
                                new Point(37.000, 137.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(37.000, 137.000, Point.CARTESIAN),
                                new Point(5.000, 134.000, Point.CARTESIAN),
                                new Point(4.000, 51.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(4.000, 51.000, Point.CARTESIAN),
                                new Point(25.000, 132.000, Point.CARTESIAN),
                                new Point(89.000, 86.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(89.000, 86.000, Point.CARTESIAN),
                                new Point(5.000, 111.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        follower.followPath(specimen);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly circular shape of radius " + RADIUS
                            + ", starting on the right-most edge. So, make sure you have enough "
                            + "space to the left, front, and back to run the OpMode.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            follower.followPath(specimen);
        }

        follower.telemetryDebug(telemetryA);
    }
}
