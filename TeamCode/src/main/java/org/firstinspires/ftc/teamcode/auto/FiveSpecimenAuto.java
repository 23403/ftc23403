package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.teleOp.MainV5.eaLimitHigh;

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

import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.CustomPedroPathing;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.extendArmStates;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;

/**
 * BeastKit V5 auto
 * Started code  @  4/13/25  @  3:30 pm
 * Expected to finish code  @  4/15/25
 * It is a 5 specimen auto with park. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.3, 4/13/25
**/

@Config("5 Spec Auto")
@Autonomous(name = "5+0", group = ".ftc23403")
public class FiveSpecimenAuto extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    public static Integer pauses = 500;
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    /** store the state of our auto. **/
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
    // extendArm
    private PIDController controller;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private double slidesTARGET = 0;
    private static extendArmStates extendArmState = extendArmStates.FLOATING;
    ElapsedTime resetTimer = new ElapsedTime();

    /** different modes **/
    private Path preload;
    /** path names **/
    private PathChain grabSpecimen1, scoreSpecimen1, moveToPushLoc1, pushBlock1, moveToPushLoc2, pushBlock2, moveToPushLoc3, pushBlock3, scoreSpecimen2, grabSpecimen2, scoreSpecimen3, grabSpecimen3, scoreSpecimen4, pushSpecimens, park;
    /** points **/
    /* start pos */
    private static final Pose startPos = new Pose(9, 63.4, Math.toRadians(0));
    /* line1 */
    public static CustomPedroPathing.beizerLine preloadPoints = new CustomPedroPathing.beizerLine(
            37.1,
            61.1,
            startPos.getX(),
            startPos.getY(),
            startPos.getHeading(),
            0
    );
    /* line2 */
    public static CustomPedroPathing.beizerLine grabSpecimen1Points = new CustomPedroPathing.beizerLine(
            17.3,
            35.8,
            preloadPoints.endPointX,
            preloadPoints.endPointY,
            preloadPoints.getEndHeading(),
            0
    );
    /* line3 */
    public static CustomPedroPathing.beizerLine scoreSpecimen1Points = new CustomPedroPathing.beizerLine(
            36.85,
            36.35,
            grabSpecimen1Points.endPointX,
            grabSpecimen1Points.endPointY,
            grabSpecimen1Points.getEndHeading(),
            0
    );
    /* line4 */
    public static CustomPedroPathing.beizerCurve moveToPushLoc1Points = new CustomPedroPathing.beizerCurve(
            scoreSpecimen1Points.endPointX,
            scoreSpecimen1Points.endPointY,
            List.of(17.94, 16.9),
            List.of(16.9, 45.2),
            54.74,
            30.05,
            scoreSpecimen1Points.getEndHeading(),
            0
    );
    /* line5 */
    public static CustomPedroPathing.beizerLine pushBlock1Points = new CustomPedroPathing.beizerLine(
            17.6,
            30.05,
            moveToPushLoc1Points.endPointX,
            moveToPushLoc1Points.endPointY,
            moveToPushLoc1Points.getEndHeading(),
            0
    );
    /* line6 */
    public static CustomPedroPathing.beizerCurve moveToPushLoc2Points = new CustomPedroPathing.beizerCurve(
            pushBlock1Points.endPointX,
            pushBlock1Points.endPointY,
            58.74,
            31,
            54.5,
            20.3,
            pushBlock1Points.getEndHeading(),
            0
    );
    /* line7 */
    public static CustomPedroPathing.beizerLine pushBlock2Points = new CustomPedroPathing.beizerLine(
            17.6,
            20.3,
            moveToPushLoc2Points.endPointX,
            moveToPushLoc2Points.endPointY,
            moveToPushLoc2Points.getEndHeading(),
            0
    );
    /* line8 */
    public static CustomPedroPathing.beizerCurve moveToPushLoc3Points = new CustomPedroPathing.beizerCurve(
            pushBlock2Points.endPointX,
            pushBlock2Points.endPointY,
            58.29,
            21.9,
            54.05,
            11.2,
            pushBlock2Points.getEndHeading(),
            0
    );
    /* line9 */
    public static CustomPedroPathing.beizerLine pushBlock3Points = new CustomPedroPathing.beizerLine(
            17.33,
            11.2,
            moveToPushLoc3Points.endPointX,
            moveToPushLoc3Points.endPointY,
            moveToPushLoc3Points.getEndHeading(),
            0
    );
    /* line10 */
    public static CustomPedroPathing.beizerCurve scoreSpecimen2Points = new CustomPedroPathing.beizerCurve(
            pushBlock3Points.endPointX,
            pushBlock3Points.endPointY,
            31.64,
            72.3,
            37,
            66.55,
            pushBlock3Points.getEndHeading(),
            0
    );
    /* line11 */
    public static CustomPedroPathing.beizerLine grabSpecimen2Points = new CustomPedroPathing.beizerLine(
            17.3,
            35.8,
            scoreSpecimen2Points.endPointX,
            scoreSpecimen2Points.endPointY,
            scoreSpecimen2Points.getEndHeading(),
            0
    );
    /* line12 */
    public static CustomPedroPathing.beizerLine scoreSpecimen3Points = new CustomPedroPathing.beizerLine(
            37.72,
            68.9,
            grabSpecimen2Points.endPointX,
            grabSpecimen2Points.endPointY,
            grabSpecimen2Points.getEndHeading(),
            0
    );
    /* line13 */
    public static CustomPedroPathing.beizerLine grabSpecimen3Points = new CustomPedroPathing.beizerLine(
            17.3,
            35.8,
            scoreSpecimen3Points.endPointX,
            scoreSpecimen3Points.endPointY,
            scoreSpecimen3Points.getEndHeading(),
            0
    );
    /* line14 */
    public static CustomPedroPathing.beizerLine scoreSpecimen4Points = new CustomPedroPathing.beizerLine(
            37.77,
            72.45,
            grabSpecimen3Points.endPointX,
            grabSpecimen3Points.endPointY,
            grabSpecimen3Points.getEndHeading(),
            0
    );
    /* line15 */
    public static CustomPedroPathing.beizerLine pushSpecimensPoints = new CustomPedroPathing.beizerLine(
            35,
            64.4,
            scoreSpecimen4Points.endPointX,
            scoreSpecimen4Points.endPointY,
            scoreSpecimen4Points.getEndHeading(),
            0
    );
    /* line16 */
    public static CustomPedroPathing.beizerLine parkPoints = new CustomPedroPathing.beizerLine(
            16.8,
            49.23,
            pushSpecimensPoints.endPointX,
            pushSpecimensPoints.endPointY,
            pushSpecimensPoints.getEndHeading(),
            -130
    );
    /** create paths **/
    public void buildPaths() {
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
        scoreSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen1Points.getStartPoint(),
                        scoreSpecimen1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen1Points.getStartHeading()), Math.toRadians(scoreSpecimen1Points.getEndHeading()))
                .build();
        /* line4 */
        moveToPushLoc1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc1Points.getStartPoint(),
                        moveToPushLoc1Points.getMiddlePoint(0),
                        moveToPushLoc1Points.getMiddlePoint(1),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getStartHeading()), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .build();
        /* line5 */
        pushBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushBlock1Points.getStartPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock1Points.getStartHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .build();
        /* line6 */
        moveToPushLoc2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc2Points.getStartPoint(),
                        moveToPushLoc2Points.getMiddlePoint(),
                        moveToPushLoc2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc2Points.getStartHeading()), Math.toRadians(moveToPushLoc2Points.getEndHeading()))
                .build();
        /* line7 */
        pushBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushBlock2Points.getStartPoint(),
                        pushBlock2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock2Points.getStartHeading()), Math.toRadians(pushBlock2Points.getEndHeading()))
                .build();
        /* line8 */
        moveToPushLoc3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc3Points.getStartPoint(),
                        moveToPushLoc3Points.getMiddlePoint(),
                        moveToPushLoc3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getStartHeading()), Math.toRadians(moveToPushLoc3Points.getEndHeading()))
                .build();
        /* line9 */
        pushBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushBlock3Points.getStartPoint(),
                        pushBlock3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock3Points.getStartHeading()), Math.toRadians(pushBlock3Points.getEndHeading()))
                .build();
        /* line10 */
        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen2Points.getStartPoint(),
                        scoreSpecimen2Points.getMiddlePoint(),
                        scoreSpecimen2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getStartHeading()), Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .build();
        /* line11 */
        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen2Points.getStartPoint(),
                        grabSpecimen2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen2Points.getStartHeading()), Math.toRadians(grabSpecimen2Points.getEndHeading()))
                .build();
        /* line12 */
        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen3Points.getStartPoint(),
                        scoreSpecimen3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen3Points.getStartHeading()), Math.toRadians(scoreSpecimen3Points.getEndHeading()))
                .build();
        /* line13 */
        grabSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen3Points.getStartPoint(),
                        grabSpecimen3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen3Points.getStartHeading()), Math.toRadians(grabSpecimen3Points.getEndHeading()))
                .build();
        /* line14 */
        scoreSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen4Points.getStartPoint(),
                        scoreSpecimen4Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen4Points.getStartHeading()), Math.toRadians(scoreSpecimen4Points.getEndHeading()))
                .build();
        /* line15 */
        pushSpecimens = follower.pathBuilder()
                .addPath(new BezierLine(
                        pushSpecimensPoints.getStartPoint(),
                        pushSpecimensPoints.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushSpecimensPoints.getStartHeading()), Math.toRadians(pushSpecimensPoints.getEndHeading()))
                .build();
        /* line16 */
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        parkPoints.getStartPoint(),
                        parkPoints.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(parkPoints.getStartHeading()), Math.toRadians(parkPoints.getEndHeading()))
                .build();
    }
    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: /* line1 */
                follower.followPath(preload,true);
                // movements
                extendArmMove(5.8);
                wrist1(0.6);
                arm(0.23);
                // if we made it there
                if(!follower.isBusy()) {
                    extendArmMove(7);
                    // if we scored
                    if (extendArmState == extendArmStates.PRESET_REACHED) {
                        // open claw first
                        claw1(0);
                        // next path
                        Timer.wait(pauses);
                        setPathState(1);
                    }
                }
                break;
            case 1: /* line2 */
                follower.followPath(grabSpecimen1,true);
                // movements
                extendArmMove(0);
                wrist1(0.4);
                arm(0.97);
                // if we made it there
                if(!follower.isBusy()) {
                    // close claw first
                    claw1(1);
                    // next path
                    Timer.wait(pauses);
                    setPathState(2);
                }
                break;
            case 2: /* line3 */
                follower.followPath(scoreSpecimen1,true);
                // movements
                extendArmMove(5.8);
                wrist1(0.6);
                arm(0.23);
                // if we made it there
                if(!follower.isBusy()) {
                    extendArmMove(7);
                    // if we scored
                    if (extendArmState == extendArmStates.PRESET_REACHED) {
                        // open claw first
                        claw1(0);
                        // next path
                        Timer.wait(pauses);
                        setPathState(3);
                    }
                }
                break;
            case 3: /* line4 */
                follower.followPath(moveToPushLoc1,true);
                // movements
                extendArmMove(0);
                submersibleArm(1);
                wrist2(0.9);
                wrist1(0.5);
                arm(0.18);
                rotation(0.52);
                // if we made it there
                if(!follower.isBusy()) {
                    // next path
                    Timer.wait(pauses);
                    setPathState(4);
                }
                break;
            case 4: /* line5 */
                follower.followPath(pushBlock1,true);
                // if we made it there
                if(!follower.isBusy()) {
                    // next path
                    Timer.wait(pauses);
                    setPathState(5);
                }
                break;
            case 5: /* line6 */
                follower.followPath(moveToPushLoc2,true);
                // if we made it there
                if(!follower.isBusy()) {
                    // next path
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 6: /* line7 */
                follower.followPath(pushBlock2,true);
                // if we made it there
                if(!follower.isBusy()) {
                    // next path
                    Timer.wait(pauses);
                    setPathState(7);
                }
                break;
            case 7: /* line8 */
                follower.followPath(moveToPushLoc3,true);
                // if we made it there
                if(!follower.isBusy()) {
                    // next path
                    Timer.wait(pauses);
                    setPathState(8);
                }
                break;
            case 8: /* line9 */
                follower.followPath(pushBlock3,true);
                // movements
                extendArmMove(0);
                wrist1(0.4);
                arm(0.97);
                // if we made it there
                if(!follower.isBusy()) {
                    // close claw first
                    claw1(1);
                    // next path
                    Timer.wait(pauses);
                    setPathState(9);
                }
                break;
            case 9: /* line10 */
                follower.followPath(scoreSpecimen2,true);
                // movements
                extendArmMove(5.8);
                wrist1(0.6);
                arm(0.23);
                // if we made it there
                if(!follower.isBusy()) {
                    extendArmMove(7);
                    // if we scored
                    if (extendArmState == extendArmStates.PRESET_REACHED) {
                        // open claw first
                        claw1(0);
                        // next path
                        Timer.wait(pauses);
                        setPathState(10);
                    }
                }
                break;
            case 10: /* line11 */
                follower.followPath(grabSpecimen2,true);
                // movements
                extendArmMove(0);
                wrist1(0.4);
                arm(0.97);
                // if we made it there
                if(!follower.isBusy()) {
                    // close claw first
                    claw1(1);
                    // next path
                    Timer.wait(pauses);
                    setPathState(11);
                }
                break;
            case 11: /* line12 */
                follower.followPath(scoreSpecimen3,true);
                // movements
                extendArmMove(5.8);
                wrist1(0.6);
                arm(0.23);
                // if we made it there
                if(!follower.isBusy()) {
                    extendArmMove(7);
                    // if we scored
                    if (extendArmState == extendArmStates.PRESET_REACHED) {
                        // open claw first
                        claw1(0);
                        // next path
                        Timer.wait(pauses);
                        setPathState(12);
                    }
                }
                break;
            case 12: /* line13 */
                follower.followPath(grabSpecimen3,true);
                // movements
                extendArmMove(0);
                wrist1(0.4);
                arm(0.97);
                // if we made it there
                if(!follower.isBusy()) {
                    // close claw first
                    claw1(1);
                    // next path
                    Timer.wait(pauses);
                    setPathState(13);
                }
                break;
            case 13: /* line14 */
                follower.followPath(scoreSpecimen4,true);
                // movements
                extendArmMove(5.8);
                wrist1(0.6);
                arm(0.23);
                // if we made it there
                if(!follower.isBusy()) {
                    extendArmMove(7);
                    // if we scored
                    if (extendArmState == extendArmStates.PRESET_REACHED) {
                        // open claw first
                        claw1(0);
                        // next path
                        Timer.wait(pauses);
                        setPathState(14);
                    }
                }
                break;
            case 14: /* line15 */
                follower.followPath(pushSpecimens,true);
                // movements
                extendArmMove(0);
                wrist1(0.4);
                arm(0.97);
                // if we made it there
                if(!follower.isBusy()) {
                    // close claw first
                    claw1(1);
                    // next path
                    Timer.wait(pauses);
                    setPathState(15);
                }
                break;
            case 15: /* line16 */
                follower.followPath(park,true);
                // movements
                extendArmMove(0);
                wrist2(0.5);
                wrist1(0.5);
                arm(0.18);
                claw2(1);
                rotation(0.52);
                Timer.wait(500);
                submersibleArm(0);
                // if we made it there
                if(!follower.isBusy()) {
                    // next path
                    setPathState(-1);
                }
                break;
        }
    }

    /** movements logic **/
    private void extendArmMove(double pos) {
        slidesTARGET = pos;
        extendArmState = extendArmStates.MOVING_TO_PRESET;
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

    /** change state of the paths and actions and reset the timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** init **/
    @Override
    public void init() {
        // hardware
        MetroLib.setConstants(MConstants.class);
        Calibrate.Auto.clearEverything();
        hardwareMap.get(IMU.class, ThreeWheelIMUConstants.IMU_HardwareMapName).resetYaw();
        controller = new PIDController(P, I, D);
        // motors
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        sweeper = hardwareMap.get(Servo.class, "sweeper"); // 1x goBilda torque
        // ea
        arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x goBilda speed
        // sa
        submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // 1x axon
        submersibleArm2 = hardwareMap.get(Servo.class, "subArm2"); // 1x axon
        wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x 25kg
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
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
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

    /** play loop **/
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
        controller.setPID(Math.sqrt(P), I, D);
        // Get current positions
        int eaTicks1 = extendArm1.getCurrentPosition();
        int eaTicks2 = extendArm2.getCurrentPosition();
        // Convert ticks to inches
        double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
        double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
        // vars
        double ff = F;
        if (Math.abs(eaInches1 - 0) > 2 && (extendArmState == extendArmStates.PRESET_REACHED || extendArmState == extendArmStates.ZERO_POS_RESET ||  extendArmState == extendArmStates.MAX_POS)) {
            extendArm1.setPower(ff);
            extendArm2.setPower(ff);
        }
        // states
        if (Math.abs(eaInches1 - eaLimitHigh) < 1 && extendArmState != extendArmStates.MOVING_TO_PRESET) {
            extendArmState = extendArmStates.MAX_POS;
        } else if (Math.abs(eaInches1 - 0) < 2 && extendArmState != extendArmStates.MOVING_TO_PRESET && extendArmState != extendArmStates.RESETTING_ZERO_POS && extendArmState != extendArmStates.ZERO_POS_RESET && extendArmState != extendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            extendArmState = extendArmStates.WAITING_FOR_RESET_CONFIRMATION;
            resetTimer.reset();
        }
        // pre resetting slides pos
        if (extendArmState == extendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            if (resetTimer.milliseconds() > 200 && Math.abs(eaInches1 - 0) < 2) {
                extendArmState = extendArmStates.RESETTING_ZERO_POS;
                resetTimer.reset();
            }
        }
        // reset slides 0 pos
        if (extendArmState == extendArmStates.RESETTING_ZERO_POS) {
            if (resetTimer.milliseconds() < 200) {
                extendArm1.setPower(-0.1);
                extendArm2.setPower(-0.1);
            } else {
                extendArm1.setPower(0);
                extendArm2.setPower(0);
                Motors.resetEncoders(List.of(extendArm1, extendArm2));
                Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                extendArmState = extendArmStates.ZERO_POS_RESET;
            }
        }
        // preset controls
        if (extendArmState == extendArmStates.MOVING_TO_PRESET) {
            double pid = controller.calculate(eaInches1, slidesTARGET);
            double rawPower = pid + ff;
            double syncError = eaInches1 - eaInches2;
            double correction = syncError * K;
            extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
            extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
            // check if we are at the target by 50 encoders
            if (Math.abs(eaInches1 - slidesTARGET) < 1) {
                extendArmState = extendArmStates.PRESET_REACHED;
            }
        }
        // telemetry for debugging
        telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
        telemetry.addData("target", slidesTARGET);
        telemetry.addData("eaCpos1", eaInches1);
        telemetry.addData("eaCpos2", eaInches2);
        telemetry.addData("eaPower", extendArm1.getPower());
        telemetry.addData("error1", Math.abs(slidesTARGET - eaInches1));
        telemetry.addData("error2", Math.abs(slidesTARGET - eaInches2));
        telemetry.addData("errorAvg", (Math.abs(slidesTARGET - eaInches1) + Math.abs(slidesTARGET - eaInches2)) / 2);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** init loop **/
    @Override
    public void init_loop() {}

    /** idk **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** stop **/
    @Override
    public void stop() {
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}

