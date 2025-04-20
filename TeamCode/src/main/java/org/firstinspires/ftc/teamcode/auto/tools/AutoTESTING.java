package org.firstinspires.ftc.teamcode.auto.tools;

import static org.firstinspires.ftc.teamcode.teleOp.MainV5.eaLimitHigh;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.CustomPedroPathing;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Test out points using the dashboard without having to upload code.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.3, 4/17/25
**/

@Config("Auto Testing")
@Autonomous(name = "Auto Testing", group = "tools_ftc23403")
public class AutoTESTING extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    public static Integer pauses = 200;
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
    private static ExtendArmStates extendArmState = ExtendArmStates.FLOATING;
    ElapsedTime resetTimer = new ElapsedTime();
    // Get current positions
    int eaTicks1 = 0;
    int eaTicks2 = 0;
    // Convert ticks to inches
    double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
    double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
    /* preload lines */
    boolean preloadStarted = false;
    boolean grabSpecimen1Started = false;
    boolean scoreSpecimen1Started = false;
    boolean pushBlocksStarted = false;
    boolean pushBlocksEnded = false;
    boolean scoreSpecimen2Started = false;
    boolean grabSpecimen2Started = false;
    boolean scoreSpecimen3Started = false;
    boolean parkStarted = false;

    /** different modes **/
    private Path preload;
    /** path names **/
    private PathChain grabSpecimen1, scoreSpecimen1, pushBlocks, scoreSpecimen2, grabSpecimen2, scoreSpecimen3, park;
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
            21.65,
            35.8,
            preloadPoints.endPointX,
            preloadPoints.endPointY,
            preloadPoints.getEndHeading(),
            0
    );
    /* line3 */
    public static CustomPedroPathing.beizerLine scoreSpecimen1Points = new CustomPedroPathing.beizerLine(
            36.85,
            62.35,
            grabSpecimen1Points.endPointX,
            grabSpecimen1Points.endPointY,
            grabSpecimen1Points.getEndHeading(),
            0
    );
    /* line4a */
    public static CustomPedroPathing.beizerCurve moveToPushLoc1Points = new CustomPedroPathing.beizerCurve(
            scoreSpecimen1Points.endPointX,
            scoreSpecimen1Points.endPointY,
            List.of(27.5, 55.2),
            List.of(21.8, 45.1),
            54.74,
            28,
            scoreSpecimen1Points.getEndHeading(),
            0
    );
    /* line4b */
    public static CustomPedroPathing.beizerLine pushBlock1Points = new CustomPedroPathing.beizerLine(
            17.6,
            28,
            moveToPushLoc1Points.endPointX,
            moveToPushLoc1Points.endPointY,
            moveToPushLoc1Points.getEndHeading(),
            0
    );
    /* line4c */
    public static CustomPedroPathing.beizerCurve moveToPushLoc2Points = new CustomPedroPathing.beizerCurve(
            pushBlock1Points.endPointX,
            pushBlock1Points.endPointY,
            58.74,
            31,
            53.94,
            20.3,
            pushBlock1Points.getEndHeading(),
            0
    );
    /* line4d */
    public static CustomPedroPathing.beizerLine pushBlock2Points = new CustomPedroPathing.beizerLine(
            17.6,
            20.3,
            moveToPushLoc2Points.endPointX,
            moveToPushLoc2Points.endPointY,
            moveToPushLoc2Points.getEndHeading(),
            0
    );
    /* line4e */
    public static CustomPedroPathing.beizerCurve moveToPushLoc3Points = new CustomPedroPathing.beizerCurve(
            pushBlock2Points.endPointX,
            pushBlock2Points.endPointY,
            53.49,
            22.5,
            54.05,
            13.3,
            pushBlock2Points.getEndHeading(),
            0
    );
    /* line4f */
    public static CustomPedroPathing.beizerLine pushBlock3Points = new CustomPedroPathing.beizerLine(
            19.8,
            11.8,
            moveToPushLoc3Points.endPointX,
            moveToPushLoc3Points.endPointY,
            moveToPushLoc3Points.getEndHeading(),
            0
    );
    /* line10 */
    public static CustomPedroPathing.beizerLine scoreSpecimen2Points = new CustomPedroPathing.beizerLine(
            35.85,
            65.55,
            pushBlock3Points.endPointX,
            pushBlock3Points.endPointY,
            pushBlock3Points.getEndHeading(),
            0
    );
    /* line11 */
    public static CustomPedroPathing.beizerLine grabSpecimen2Points = new CustomPedroPathing.beizerLine(
            18.5,
            35.8,
            scoreSpecimen2Points.endPointX,
            scoreSpecimen2Points.endPointY,
            scoreSpecimen2Points.getEndHeading(),
            0
    );
    /* line12 */
    public static CustomPedroPathing.beizerLine scoreSpecimen3Points = new CustomPedroPathing.beizerLine(
            37,
            70,
            grabSpecimen2Points.endPointX,
            grabSpecimen2Points.endPointY,
            grabSpecimen2Points.getEndHeading(),
            0
    );
    /* line13 */
    public static CustomPedroPathing.beizerCurve parkPoints = new CustomPedroPathing.beizerCurve(
            scoreSpecimen3Points.endPointX,
            scoreSpecimen3Points.endPointY,
            27.2,
            69.3,
            16.8,
            49.23,
            scoreSpecimen3Points.getEndHeading(),
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
        pushBlocks = follower.pathBuilder()
                .addPath(new BezierCurve(
                        moveToPushLoc1Points.getStartPoint(),
                        moveToPushLoc1Points.getMiddlePoint(0),
                        moveToPushLoc1Points.getMiddlePoint(1),
                        moveToPushLoc1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc1Points.getStartHeading()), Math.toRadians(moveToPushLoc1Points.getEndHeading()))
                .addPath(new BezierLine(
                        pushBlock1Points.getStartPoint(),
                        pushBlock1Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock1Points.getStartHeading()), Math.toRadians(pushBlock1Points.getEndHeading()))
                .addPath(new BezierCurve(
                        moveToPushLoc2Points.getStartPoint(),
                        moveToPushLoc2Points.getMiddlePoint(),
                        moveToPushLoc2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc2Points.getStartHeading()), Math.toRadians(moveToPushLoc2Points.getEndHeading()))
                .addPath(new BezierLine(
                        pushBlock2Points.getStartPoint(),
                        pushBlock2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock2Points.getStartHeading()), Math.toRadians(pushBlock2Points.getEndHeading()))
                .addPath(new BezierCurve(
                        moveToPushLoc3Points.getStartPoint(),
                        moveToPushLoc3Points.getMiddlePoint(),
                        moveToPushLoc3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(moveToPushLoc3Points.getStartHeading()), Math.toRadians(moveToPushLoc3Points.getEndHeading()))
                .addPath(new BezierLine(
                        pushBlock3Points.getStartPoint(),
                        pushBlock3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(pushBlock3Points.getStartHeading()), Math.toRadians(pushBlock3Points.getEndHeading()))
                .build();
        /* line5 */
        scoreSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scoreSpecimen2Points.getStartPoint(),
                        scoreSpecimen2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen2Points.getStartHeading()), Math.toRadians(scoreSpecimen2Points.getEndHeading()))
                .build();
        /* line6 */
        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        grabSpecimen2Points.getStartPoint(),
                        grabSpecimen2Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(grabSpecimen2Points.getStartHeading()), Math.toRadians(grabSpecimen2Points.getEndHeading()))
                .build();
        /* line7 */
        scoreSpecimen3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        scoreSpecimen3Points.getStartPoint(),
                        scoreSpecimen3Points.getEndPoint()
                ))
                .setLinearHeadingInterpolation(Math.toRadians(scoreSpecimen3Points.getStartHeading()), Math.toRadians(scoreSpecimen3Points.getEndHeading()))
                .build();
        /* line8 */
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        parkPoints.getStartPoint(),
                        parkPoints.getEndPoint()
                ))
                .setConstantHeadingInterpolation(Math.toRadians(parkPoints.getEndHeading()))
                .build();
    }
    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: /* line1 */
                if (!preloadStarted) {
                    extendArmMove(10);
                    submersibleArm(1);
                    wrist1(0.6);
                    arm(0.23);
                    follower.followPath(preload, false);
                    preloadStarted = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        extendArmMove(19);
                    } else if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(1);
                    }
                }
                break;
            case 1: /* line2 */
                if (!grabSpecimen1Started) {
                    extendArmMove(0);
                    wrist1(0.42);
                    arm(0.96);
                    follower.followPath(grabSpecimen1, false);
                    grabSpecimen1Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(2);
                }
                break;
            case 2: /* line3 */
                if (!scoreSpecimen1Started) {
                    extendArmMove(10);
                    wrist1(0.6);
                    arm(0.23);
                    follower.followPath(scoreSpecimen1, false);
                    scoreSpecimen1Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        extendArmMove(19);
                    } else if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(3);
                    }
                }
                break;
            case 3: /* line4 */
                if (!pushBlocksStarted) {
                    extendArmMove(0);
                    submersibleArm(1);
                    wrist2(0.9);
                    wrist1(0.5);
                    arm(0.18);
                    rotation(0.52);
                    follower.followPath(pushBlocks, true);
                    pushBlocksStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!pushBlocksEnded) {
                        extendArmMove(0);
                        wrist1(0.42);
                        arm(0.96);
                        claw1(0);
                        pushBlocksEnded = true;
                    } else {
                        claw1.setPosition(1);
                        claw1(1);
                        Timer.wait(pauses);
                        setPathState(4);
                    }
                }
                break;
            case 4: /* line5 */
                if (!scoreSpecimen2Started) {
                    extendArmMove(10);
                    wrist1(0.6);
                    arm(0.23);
                    follower.followPath(scoreSpecimen2, false);
                    scoreSpecimen2Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        extendArmMove(19);
                    } else if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(5);
                    }
                }
                break;
            case 5: /* line6 */
                if (!grabSpecimen2Started) {
                    extendArmMove(0);
                    wrist1(0.42);
                    arm(0.96);
                    follower.followPath(grabSpecimen2, false);
                    grabSpecimen2Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 6: /* line7 */
                if (!scoreSpecimen3Started) {
                    extendArmMove(10);
                    wrist1(0.6);
                    arm(0.23);
                    follower.followPath(scoreSpecimen3, false);
                    scoreSpecimen3Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        extendArmMove(19);
                    } else if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(7);
                    }
                }
                break;
            case 7: /* line8 */
                if (!parkStarted) {
                    claw1(0);
                    extendArmMove(0);
                    wrist2(0.5);
                    wrist1(0.5);
                    arm(0.18);
                    claw2(1);
                    rotation(0.52);
                    Timer.wait(500);
                    follower.followPath(park, false);
                    parkStarted = true;
                }
                if (!follower.isBusy() && parkStarted) {
                    submersibleArm(0);
                    setPathState(-1);
                }
                break;
        }
    }

    /** movements logic **/
    private void extendArmMove(double pos) {
        slidesTARGET = pos;
        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
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
        controller = new PIDController(Math.sqrt(P), I, D);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
        submersibleArm1.scaleRange(0.42, 1);
        // extendArm
        Motors.resetEncoders(List.of(extendArm1, extendArm2));
        Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        resetTimer.reset();
        // starting pos
        claw1.setPosition(1);
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
        eaTicks1 = extendArm1.getCurrentPosition();
        eaTicks2 = extendArm2.getCurrentPosition();
        // Convert ticks to inches
        eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
        eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
        // vars
        double ff = F;
        if (Math.abs(eaInches1 - 0) > 2 && (extendArmState == ExtendArmStates.PRESET_REACHED || extendArmState == ExtendArmStates.ZERO_POS_RESET ||  extendArmState == ExtendArmStates.MAX_POS)) {
            extendArm1.setPower(ff);
            extendArm2.setPower(ff);
        }
        // states
        if (Math.abs(eaInches1 - eaLimitHigh) < 1 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
            extendArmState = ExtendArmStates.MAX_POS;
        } else if (Math.abs(eaInches1 - 0) < 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET && extendArmState != ExtendArmStates.RESETTING_ZERO_POS && extendArmState != ExtendArmStates.ZERO_POS_RESET && extendArmState != ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            extendArmState = ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION;
            resetTimer.reset();
        }
        // pre resetting slides pos
        if (extendArmState == ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            if (resetTimer.milliseconds() > 200 && Math.abs(eaInches1 - 0) < 2) {
                extendArmState = ExtendArmStates.RESETTING_ZERO_POS;
                resetTimer.reset();
            }
        }
        // reset slides 0 pos
        if (extendArmState == ExtendArmStates.RESETTING_ZERO_POS) {
            if (resetTimer.milliseconds() < 200) {
                extendArm1.setPower(-0.1);
                extendArm2.setPower(-0.1);
            } else {
                extendArm1.setPower(0);
                extendArm2.setPower(0);
                Motors.resetEncoders(List.of(extendArm1, extendArm2));
                Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                extendArmState = ExtendArmStates.ZERO_POS_RESET;
            }
        }
        // preset controls
        if (extendArmState == ExtendArmStates.MOVING_TO_PRESET) {
            double pid = controller.calculate(eaInches1, slidesTARGET);
            double rawPower = pid + ff;
            double syncError = eaInches1 - eaInches2;
            double correction = syncError * K;
            extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
            extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
            // check if we are at the target by 50 encoders
            if (Math.abs(eaInches1 - slidesTARGET) < 1) {
                extendArmState = ExtendArmStates.PRESET_REACHED;
            }
        }
        // telemetry for debugging
        telemetry.addData("currentState", extendArmState);
        telemetry.addData("extendArm1 Power", extendArm1.getPower());
        telemetry.addData("extendArm2 Power", extendArm2.getPower());
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

