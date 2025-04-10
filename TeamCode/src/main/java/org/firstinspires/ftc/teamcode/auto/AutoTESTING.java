package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import xyz.nin1275.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides;
import org.firstinspires.ftc.teamcode.utils.CustomPedroPathing;
import org.firstinspires.ftc.teamcode.variables.constants.AutoVariables;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Started code  @  3/28/25  @  6:20 pm
 * Expected to finish code  @  3/29/25
 * It is a 1+4 specimen auto with park. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.2, 3/29/25
 */

@Config("Auto Testing")
@Autonomous(name = "Auto Testing", group = ".ftc23403")
public class AutoTESTING extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 0.8;
    public static Integer pauses = 1000;
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    /** store the state of our auto. */
    private int pathState;
    // servos
    private Servo sweeper; // 1x goBilda torque
    private Servo arm; // 2x axon
    private Servo wrist1; // 1x axon
    private Servo claw1; // 1x axon
    private Servo submersibleArm1; // 1x axon
    private Servo submersibleArm2; // 1x axon
    private Servo wrist2; // 1x 20kg
    private Servo claw2; // 1x goBilda speed
    private Servo rotation; // 1x goBilda speed
    // servo positions
    public static double wristCpos1 = 0;
    public static double clawCpos1 = 1;
    public static double sweeperCpos = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 0.5;
    public static double armCpos = 0.23;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0.5;
    // pid
    private PIDController controller;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private int slidesTARGET = 0;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathChains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** line positions */
    private final Pose startPos = new Pose(-9, 63.4, Math.toRadians(-90)); // start Pos
    /** different modes */
    private Path preload;
    /** path name */
    private PathChain grabSpecimen1, grabSpecimen2, scoreSpecimen1, moveToPushLoc1, pushBlock1, moveToPushLoc2, pushBlock2, moveToPushLoc3, pushBlock3, scoreSpecimen2, grabSpecimen3, grabSpecimen4, scoreSpecimen3, grabSpecimen5, grabSpecimen6, scoreSpecimen4, grabSpecimen7, grabSpecimen8, scoreSpecimen5, pushSpecimens, park;


    /** points */
    /* line1 */
    public static CustomPedroPathing.beizerLine preloadPoints = new CustomPedroPathing.beizerLine(
            10.6,
            32.2,
            -9,
            63.4,
            -90,
            -90
    );
    /* line2 */
    public static CustomPedroPathing.beizerLine grabSpecimen1Points = new CustomPedroPathing.beizerLine(
            -46,
            54.8,
            preloadPoints.endPointX,
            preloadPoints.endPointY,
            preloadPoints.getEndHeading(),
            -90
    );
    /* line3 */
    public static CustomPedroPathing.beizerLine grabSpecimen2Points = new CustomPedroPathing.beizerLine(
            -46,
            55.6,
            grabSpecimen1Points.endPointX,
            grabSpecimen1Points.endPointY,
            grabSpecimen1Points.getEndHeading(),
            -90
    );
    /* line4 */
    public static CustomPedroPathing.beizerLine scoreSpecimen1Points = new CustomPedroPathing.beizerLine(
            2.8,
            31.2,
            grabSpecimen2Points.endPointX,
            grabSpecimen2Points.endPointY,
            grabSpecimen2Points.getEndHeading(),
            -90
    );
    /* line5 */
    public static CustomPedroPathing.beizerCurve moveToPushLoc1Points = new CustomPedroPathing.beizerCurve(
            scoreSpecimen1Points.endPointX,
            scoreSpecimen1Points.endPointY,
            List.of(0.0),
            List.of(0.0),
            2.8,
            31.2,
            scoreSpecimen1Points.getEndHeading(),
            -90
    );
    /* line6 */
    public static CustomPedroPathing.beizerLine pushBlock1Points = new CustomPedroPathing.beizerLine(
            -47.3,
            54.6,
            moveToPushLoc1Points.endPointX,
            moveToPushLoc1Points.endPointY,
            moveToPushLoc1Points.getEndHeading(),
            -90
    );
    /* line7 */
    public static CustomPedroPathing.beizerCurve moveToPushLoc2Points = new CustomPedroPathing.beizerCurve(
            pushBlock1Points.endPointX,
            pushBlock1Points.endPointY,
            List.of(0.0),
            List.of(0.0),
            -59.3,
            19.7,
            pushBlock1Points.getEndHeading(),
            -90
    );
    /* line8 */
    public static CustomPedroPathing.beizerLine pushBlock2Points = new CustomPedroPathing.beizerLine(
            -59.3,
            54.6,
            moveToPushLoc2Points.endPointX,
            moveToPushLoc2Points.endPointY,
            moveToPushLoc2Points.getEndHeading(),
            -90
    );
    /* line9 */
    public static CustomPedroPathing.beizerCurve moveToPushLoc3Points = new CustomPedroPathing.beizerCurve(
            pushBlock2Points.endPointX,
            pushBlock2Points.endPointY,
            List.of(0.0),
            List.of(0.0),
            -65.9,
            20.5,
            pushBlock2Points.getEndHeading(),
            -90
    );
    /* line10 */
    public static CustomPedroPathing.beizerLine pushBlock3Points = new CustomPedroPathing.beizerLine(
            -64.6,
            57.7,
            moveToPushLoc3Points.endPointX,
            moveToPushLoc3Points.endPointY,
            moveToPushLoc3Points.getEndHeading(),
            -90
    );
    /* line11 */
    public static CustomPedroPathing.beizerLine scoreSpecimen2Points = new CustomPedroPathing.beizerLine(
            -1.5,
            33.3,
            pushBlock3Points.endPointX,
            pushBlock3Points.endPointY,
            pushBlock3Points.getEndHeading(),
            -90
    );
    /* line12 */
    public static CustomPedroPathing.beizerLine grabSpecimen3Points = new CustomPedroPathing.beizerLine(
            -46,
            54.8,
            scoreSpecimen2Points.endPointX,
            scoreSpecimen2Points.endPointY,
            scoreSpecimen2Points.getEndHeading(),
            -90
    );
    /* line13 */
    public static CustomPedroPathing.beizerLine grabSpecimen4Points = new CustomPedroPathing.beizerLine(
            -46,
            55.6,
            grabSpecimen3Points.endPointX,
            grabSpecimen3Points.endPointY,
            grabSpecimen3Points.getEndHeading(),
            -90
    );
    /* line14 */
    public static CustomPedroPathing.beizerLine scoreSpecimen3Points = new CustomPedroPathing.beizerLine(
            -4.4,
            33.4,
            grabSpecimen4Points.endPointX,
            grabSpecimen4Points.endPointY,
            grabSpecimen4Points.getEndHeading(),
            -90
    );
    /* line15 */
    public static CustomPedroPathing.beizerLine grabSpecimen5Points = new CustomPedroPathing.beizerLine(
            -46,
            54.8,
            scoreSpecimen3Points.endPointX,
            scoreSpecimen3Points.endPointY,
            scoreSpecimen3Points.getEndHeading(),
            -90
    );
    /* line16 */
    public static CustomPedroPathing.beizerLine grabSpecimen6Points = new CustomPedroPathing.beizerLine(
            -46,
            55.6,
            grabSpecimen5Points.endPointX,
            grabSpecimen5Points.endPointY,
            grabSpecimen5Points.getEndHeading(),
            -90
    );
    /* line17 */
    public static CustomPedroPathing.beizerLine scoreSpecimen4Points = new CustomPedroPathing.beizerLine(
            -7,
            33.4,
            grabSpecimen6Points.endPointX,
            grabSpecimen6Points.endPointY,
            grabSpecimen6Points.getEndHeading(),
            -90
    );
    /* line18 */
    public static CustomPedroPathing.beizerLine grabSpecimen7Points = new CustomPedroPathing.beizerLine(
            -46,
            54.8,
            scoreSpecimen4Points.endPointX,
            scoreSpecimen4Points.endPointY,
            scoreSpecimen4Points.getEndHeading(),
            -90
    );
    /* line19 */
    public static CustomPedroPathing.beizerLine grabSpecimen8Points = new CustomPedroPathing.beizerLine(
            -46,
            55.6,
            grabSpecimen7Points.endPointX,
            grabSpecimen7Points.endPointY,
            grabSpecimen7Points.getEndHeading(),
            -90
    );
    /* line20 */
    public static CustomPedroPathing.beizerLine scoreSpecimen5Points = new CustomPedroPathing.beizerLine(
            -9,
            33.6,
            grabSpecimen8Points.endPointX,
            grabSpecimen8Points.endPointY,
            grabSpecimen8Points.getEndHeading(),
            -90
    );
    /* line21 */
    public static CustomPedroPathing.beizerLine pushSpecimensPoints = new CustomPedroPathing.beizerLine(
            -0.6,
            33.3,
            scoreSpecimen5Points.endPointX,
            scoreSpecimen5Points.endPointY,
            scoreSpecimen5Points.getEndHeading(),
            -90
    );
    /* line22 */
    public static CustomPedroPathing.beizerLine parkPoints = new CustomPedroPathing.beizerLine(
            -27,
            53.6,
            pushSpecimensPoints.endPointX,
            pushSpecimensPoints.endPointY,
            pushSpecimensPoints.getEndHeading(),
            160
    );
    /** create paths */
    public void buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* line1 */
        preload = new Path(
                new BezierLine(
                preloadPoints.getStartPoint(),
                preloadPoints.getEndPoint()
        ));
        preload.setLinearHeadingInterpolation(Math.toRadians(preloadPoints.getStartHeading()), Math.toRadians(preloadPoints.getEndHeading()));
        /* line2 */
        grabSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen1Points.getStartPoint(),
                        grabSpecimen1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen1Points.getStartHeading()), Math.toRadians(grabSpecimen1Points.getEndHeading()))
                .build();
        /* line3 */
        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen2Points.getStartPoint(),
                        grabSpecimen2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen2Points.getStartHeading()), Math.toRadians(grabSpecimen2Points.getEndHeading()))
                .build();
        /* line4 */
        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen1Points.getStartPoint(),
                        scoreSpecimen1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getStartHeading()), Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .build();
        /* line5 */
        moveToPushLoc1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc1Points.getStartPoint(),
                        moveToPushLoc1Points.getMiddlePoint(0),
                        moveToPushLoc1Points.getMiddlePoint(0),
                        moveToPushLoc1Points.getMiddlePoint(1),
                        moveToPushLoc1Points.getMiddlePoint(1),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getStartHeading()), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .build();
        /* line6 */
        pushBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushBlock1Points.getStartPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock1Points.getStartHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .build();
        /* line7 */
        moveToPushLoc2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc2Points.getStartPoint(),
                        moveToPushLoc2Points.getMiddlePoint(0),
                        moveToPushLoc2Points.getMiddlePoint(0),
                        moveToPushLoc2Points.getMiddlePoint(1),
                        moveToPushLoc2Points.getMiddlePoint(1),
                        moveToPushLoc2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc2Points.getStartHeading()), Math.toRadians(moveToPushLoc2Points.getEndHeading()))
                .build();
        /* line8 */
        pushBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushBlock2Points.getStartPoint(),
                        pushBlock2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock2Points.getStartHeading()), Math.toRadians(pushBlock2Points.getEndHeading()))
                .build();
        /* line9 */
        moveToPushLoc3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc3Points.getStartPoint(),
                        moveToPushLoc3Points.getMiddlePoint(0),
                        moveToPushLoc3Points.getMiddlePoint(0),
                        moveToPushLoc3Points.getMiddlePoint(1),
                        moveToPushLoc3Points.getMiddlePoint(1),
                        moveToPushLoc3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getStartHeading()), Math.toRadians(moveToPushLoc3Points.getEndHeading()))
                .build();
        /* line10 */
        pushBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushBlock3Points.getStartPoint(),
                        pushBlock3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock3Points.getStartHeading()), Math.toRadians(pushBlock3Points.getEndHeading()))
                .build();
        /* line11 */
        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen2Points.getStartPoint(),
                        scoreSpecimen2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getStartHeading()), Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .build();
        /* line12 */
        grabSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen3Points.getStartPoint(),
                        grabSpecimen3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen3Points.getStartHeading()), Math.toRadians(grabSpecimen3Points.getEndHeading()))
                .build();
        /* line13 */
        grabSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen4Points.getStartPoint(),
                        grabSpecimen4Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen4Points.getStartHeading()), Math.toRadians(grabSpecimen4Points.getEndHeading()))
                .build();
        /* line14 */
        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen3Points.getStartPoint(),
                        scoreSpecimen3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen3Points.getStartHeading()), Math.toRadians(scoreSpecimen3Points.getEndHeading()))
                .build();
        /* line15 */
        grabSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen5Points.getStartPoint(),
                        grabSpecimen5Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen5Points.getStartHeading()), Math.toRadians(grabSpecimen5Points.getEndHeading()))
                .build();
        /* line16 */
        grabSpecimen6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen6Points.getStartPoint(),
                        grabSpecimen6Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen6Points.getStartHeading()), Math.toRadians(grabSpecimen6Points.getEndHeading()))
                .build();
        /* line17 */
        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen4Points.getStartPoint(),
                        scoreSpecimen4Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen4Points.getStartHeading()), Math.toRadians(scoreSpecimen4Points.getEndHeading()))
                .build();
        /* line18 */
        grabSpecimen7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen7Points.getStartPoint(),
                        grabSpecimen7Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen7Points.getStartHeading()), Math.toRadians(grabSpecimen7Points.getEndHeading()))
                .build();
        /* line19 */
        grabSpecimen8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen8Points.getStartPoint(),
                        grabSpecimen8Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen8Points.getStartHeading()), Math.toRadians(grabSpecimen8Points.getEndHeading()))
                .build();
        /* line20 */
        scoreSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen5Points.getStartPoint(),
                        scoreSpecimen5Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen5Points.getStartHeading()), Math.toRadians(scoreSpecimen5Points.getEndHeading()))
                .build();
        /* line21 */
        pushSpecimens = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushSpecimensPoints.getStartPoint(),
                        pushSpecimensPoints.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushSpecimensPoints.getStartHeading()), Math.toRadians(pushSpecimensPoints.getEndHeading()))
                .build();
        /* line22 */
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        parkPoints.getStartPoint(),
                        parkPoints.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(parkPoints.getStartHeading()), Math.toRadians(parkPoints.getEndHeading()))
                .build();
    }

    /* You could check for
        - Follower State: "if(!follower.isBusy() {}"
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
        - Robot Position: "if(follower.getPose().getX() > 36) {}"
    */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: /* line1 */
                if(follower.getPose().getX() > 39.5) {
                    // arm movements
                }
                if(!follower.isBusy()) {
                    follower.followPath(preload,false);
                    Timer.wait(pauses);
                    setPathState(1);
                }
                break;
            case 1: /* line2 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen1,false);
                    Timer.wait(pauses);
                    setPathState(2);
                }
                break;
            case 2: /* line3 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen2,false);
                    Timer.wait(pauses);
                    setPathState(3);
                }
                break;
            case 3: /* line4 */
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpecimen1,false);
                    Timer.wait(pauses);
                    setPathState(4);
                }
                break;
            case 4: /* line5 */
                if(!follower.isBusy()) {
                    follower.followPath(moveToPushLoc1,false);
                    Timer.wait(pauses);
                    setPathState(5);
                }
                break;
            case 5: /* line6 */
                if(!follower.isBusy()) {
                    follower.followPath(pushBlock1,false);
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 6: /* line7 */
                if(!follower.isBusy()) {
                    follower.followPath(moveToPushLoc2,false);
                    Timer.wait(pauses);
                    setPathState(7);
                }
                break;
            case 7: /* line8 */
                if(!follower.isBusy()) {
                    follower.followPath(pushBlock2,false);
                    Timer.wait(pauses);
                    setPathState(8);
                }
                break;
            case 8: /* line9 */
                if(!follower.isBusy()) {
                    follower.followPath(moveToPushLoc3,false);
                    Timer.wait(pauses);
                    setPathState(9);
                }
                break;
            case 9: /* line10 */
                if(!follower.isBusy()) {
                    follower.followPath(pushBlock3,false);
                    Timer.wait(pauses);
                    setPathState(10);
                }
                break;
            case 10: /* line11 */
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpecimen2,false);
                    Timer.wait(pauses);
                    setPathState(11);
                }
                break;
            case 11: /* line12 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen3,false);
                    Timer.wait(pauses);
                    setPathState(12);
                }
                break;
            case 12: /* line13 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen4,false);
                    Timer.wait(pauses);
                    setPathState(13);
                }
                break;
            case 13: /* line14 */
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpecimen3,false);
                    Timer.wait(pauses);
                    setPathState(14);
                }
                break;
            case 14: /* line15 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen5,false);
                    Timer.wait(pauses);
                    setPathState(15);
                }
                break;
            case 15: /* line16 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen6,false);
                    Timer.wait(pauses);
                    setPathState(16);
                }
                break;
            case 16: /* line17 */
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpecimen4,false);
                    Timer.wait(pauses);
                    setPathState(17);
                }
                break;
            case 17: /* line18 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen7,false);
                    Timer.wait(pauses);
                    setPathState(18);
                }
                break;
            case 18: /* line19 */
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen8,false);
                    Timer.wait(pauses);
                    setPathState(19);
                }
                break;
            case 19: /* line20 */
                if(!follower.isBusy()) {
                    follower.followPath(scoreSpecimen5,false);
                    Timer.wait(pauses);
                    setPathState(19);
                }
                break;
            case 20: /* line21 */
                if(!follower.isBusy()) {
                    follower.followPath(pushSpecimens,false);
                    Timer.wait(pauses);
                    setPathState(19);
                }
                break;
            case 21: /* line22 */
                if(!follower.isBusy()) {
                    follower.followPath(park,false);
                    Timer.wait(pauses);
                    setPathState(19);
                }
                break;
        }
    }

    /**
     * Servo movements, and encoder movements for the slides
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    private void extendArmMove(int pos) {
        slidesTARGET = pos;
    }
    private void extendArmMoveOLD(int dis1, int dis2, double power) {
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // stuff
        AutoVariables.eaMovements1+=dis1;
        AutoVariables.eaMovements2+=dis2;
        // formula
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        extendArm1.setTargetPosition(dis1);
        extendArm2.setTargetPosition(dis2);
        // move moters
        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        int eaCpos1 = 0;
        int eaCpos2 = 0;
        while (extendArm1.isBusy() || extendArm2.isBusy()) {
            telemetry.addData("ExtendArm1Pos:", extendArm1.getCurrentPosition());
            telemetry.addData("ExtendArm2Pos:", extendArm2.getCurrentPosition());
            telemetry.update();
            eaCpos1 = extendArm1.getCurrentPosition();
            eaCpos2 = extendArm2.getCurrentPosition();
        }
        // stop
        extendArm1.setTargetPosition(eaCpos1);
        extendArm2.setTargetPosition(eaCpos2);
        extendArm1.setPower(1);
        extendArm2.setPower(1);
        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // servos
    private void claw1(double pos) {
        clawCpos1 = pos;
    }
    private void claw2(double pos) {
        clawCpos2 = pos;
    }
    private void wrist1(double pos) {
        wristCpos1 = pos;
    }
    private void wrist2(double pos) {
        wristCpos2 = pos;
    }
    private void sweeper(double pos) {
        sweeperCpos = pos;
    }
    private void arm(double pos) {
        armCpos = pos;
    }
    private void submersibleArm(double pos) {
        subArmCpos = pos;
    }
    private void rotation(double pos) {
        rotationalCpos = pos;
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        follower.setMaxPower(speed);
        autonomousPathUpdate();
        // Draw the robot on the dashboard
        poseUpdater.update();
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        // servos
        wrist1.setPosition(wristCpos1);
        wrist2.setPosition(wristCpos2);
        claw1.setPosition(clawCpos1);
        claw2.setPosition(clawCpos2);
        arm.setPosition(armCpos);
        submersibleArm1.setPosition(subArmCpos);
        sweeper.setPosition(sweeperCpos);
        rotation.setPosition(rotationalCpos);
        // extendArm code
        controller.setPID(PIDTuneSlides.P, PIDTuneSlides.I, PIDTuneSlides.D);
        int eaCpos1 = extendArm1.getCurrentPosition();
        int eaCpos2 = extendArm2.getCurrentPosition();
        double pid = controller.calculate(eaCpos1, slidesTARGET);
        double ff = PIDTuneSlides.F;
        double power = pid + ff;
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        // Feedback to Driver Hub
        telemetry.addData("PIDF", "P: " + PIDTuneSlides.P + " I: " + PIDTuneSlides.I + " D: " + PIDTuneSlides.D + " F: " + PIDTuneSlides.F);
        telemetry.addData("target", slidesTARGET);
        telemetry.addData("eaCpos1", eaCpos1);
        telemetry.addData("eaCpos2", eaCpos2);
        telemetry.addData("eaPower", power);
        telemetry.addData("error1", Math.abs(slidesTARGET - eaCpos1));
        telemetry.addData("error2", Math.abs(slidesTARGET - eaCpos2));
        telemetry.addData("errorAvg", (Math.abs(slidesTARGET - eaCpos1) + Math.abs(slidesTARGET - eaCpos2)) / 2);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        // hardware
        MetroLib.setConstants(MConstants.class);
        Calibrate.Auto.clearEverything();
        hardwareMap.get(IMU.class, ThreeWheelIMUConstants.IMU_HardwareMapName).resetYaw();
        controller = new PIDController(PIDTuneSlides.P, PIDTuneSlides.I, PIDTuneSlides.D);
        // motors
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        sweeper = hardwareMap.get(Servo.class, "sweeper"); // 1x goBilda torque
        // ea
        arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x axon
        // sa
        submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // 1x axon
        submersibleArm2 = hardwareMap.get(Servo.class, "subArm2"); // 1x axon
        wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x 20kg
        claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // directions
        sweeper.setDirection(Servo.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0, 0.8);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0.2, 1);
        claw1.scaleRange(0.4, 0.8);
        submersibleArm1.scaleRange(0.45, 1);
        // starting pos
        claw1(1);
        // movement
        pathTimer = new com.pedropathing.util.Timer();
        opmodeTimer = new com.pedropathing.util.Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPos);
        buildPaths();
        // Draw the robot on the dashboard
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(startPos);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}

