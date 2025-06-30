package org.firstinspires.ftc.teamcode.auto.tools;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.custom.PPPoint;
import xyz.nin1275.subsystems.SlidesSS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleOp.MainV6;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.CombinedServo;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Test out points using the dashboard without having to upload code.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.4, 6/30/25
**/

@Config("P2P")
@Autonomous(name = "P2P", group = "tools_ftc23403")
public class P2P extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 0.8;
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    /** store the state of our auto. */
    private int pathState;
    // servos
    // ea
    CachingServo arm1; // 1x axon max
    CachingServo arm2; // 1x axon max
    private static CachingServo wrist1; // 1x axon mini
    private static CachingServo claw1; // 1x axon mini
    private static CachingServo rotation1; // 1x axon max
    private static CombinedServo arm; // 2x axon max
    // sa
    CachingServo submersibleArm1; // 1x axon mini
    CachingServo submersibleArm2; // 1x axon mini
    private static CachingServo wrist2; // 1x axon mini
    private static CachingServo claw2; // 1x goBilda speed
    private static CachingServo rotation2; // 1x axon max
    private static CombinedServo subArm; // 2x axon mini
    // servo positions
    public static double wristCpos1 = 1;
    public static double clawCpos1 = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.15;
    public static double subArmCpos = 1;
    public static double rotationalCpos1 = 0;
    public static double rotationalCpos2 = 0;
    // extend arm
    public static double slidesTARGET = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    private CachingDcMotorEx extendArm1;
    private CachingDcMotorEx extendArm2;
    // start pos
    public static double startPosX = 7.6;
    public static double startPosY = 63.9;
    public static double startPosRotation = 0;
    // rand
    boolean pointStarted = false;
    // movements
    public static CustomPresets movement = new CustomPresets(
            -1.0,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0);

    /** line positions */
    private static final Pose startPos = new Pose(startPosX, startPosY, Math.toRadians(startPosRotation)); // start Pos
    /** different modes */
    private Path point;

    /** points */
    /* line1 */
    public static PPPoint.beizerLine pointPoints = new PPPoint.beizerLine(
            38.4,
            84,
            0
    );
    /** create paths */
    public void buildPaths() {
        /* line1 */
        point = new Path(
                new BezierLine(
                        new Point(startPos.getX(), startPos.getY()),
                        pointPoints.getEndPoint()
                ));
        point.setLinearHeadingInterpolation(startPos.getHeading(), Math.toRadians(pointPoints.getEndHeading()));
    }
    /** movements **/
    public void autonomousPathUpdate() {
        if (pathState == 0) { /* line1 */
            if (!pointStarted) {
                // movements
                extendArmMove(movement.extendArm != -1.0 ? movement.extendArm : extendArmSS.getInches1());
                submersibleArm(movement.subArm != -1.0 ? movement.subArm : subArmCpos);
                claw2(movement.claw2 != -1.0 ? movement.claw2 : clawCpos2);
                wrist2(movement.wrist2 != -1.0 ? movement.wrist2 : wristCpos2);
                wrist1(movement.wrist1 != -1.0 ? movement.wrist1 : wristCpos1);
                claw1(movement.claw1 != -1.0 ? movement.claw1 : clawCpos1);
                arm(movement.arm != -1.0 ? movement.arm : armCpos);
                rotation2(movement.rotational2 != -1.0 ? movement.rotational2 : rotationalCpos2);
                rotation1(movement.rotational1 != -1.0 ? movement.rotational1 : rotationalCpos1);
                // path
                follower.followPath(point, false);
                pointStarted = true;
            }
            if (!follower.isBusy()) {
                setPathState(-1);
            }
        }
    }

    /** movements logic **/
    private static void extendArmMove(double pos) {
        extendArmSS.moveTo(pos);
    }
    // servos
    private static void claw1(double pos) {
        clawCpos1 = pos;
    }
    private static void claw2(double pos) {
        clawCpos2 = pos;
    }
    private static void wrist1(double pos) {
        wristCpos1 = pos;
    }
    private static void wrist2(double pos) {
        wristCpos2 = pos;
    }
    private static void arm(double pos) {
        armCpos = pos;
    }
    private static void submersibleArm(double pos) {
        subArmCpos = pos;
    }
    private static void rotation1(double pos) {
        rotationalCpos1 = pos;
    }
    private static void rotation2(double pos) {
        rotationalCpos2 = pos;
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
        subArm.setPosition(subArmCpos);
        rotation1.setPosition(rotationalCpos1);
        rotation2.setPosition(rotationalCpos2);
        // extendArm code
        extendArmSS.update(false, false);
        extendArmSS.setEaCorrection(eaCorrection);
        extendArmSS.setLimits(MainV6.eaLimitHigh, MainV6.eaLimitLow);
        // telemetry
        telemetry.addLine("BEASTKIT Team 23403!");
        telemetry.addData("currentState", extendArmSS.getState());
        telemetry.addData("extendArm1 Power", extendArm1.getPower());
        telemetry.addData("extendArm2 Power", extendArm2.getPower());
        telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
        telemetry.addData("target", slidesTARGET);
        telemetry.addData("eaCpos1", extendArmSS.getInches1());
        telemetry.addData("eaCpos2", extendArmSS.getInches2());
        telemetry.addData("error1", Math.abs(slidesTARGET - extendArmSS.getInches1()));
        telemetry.addData("error2", Math.abs(slidesTARGET - extendArmSS.getInches2()));
        telemetry.addData("errorAvg", (Math.abs(slidesTARGET - extendArmSS.getInches1()) + Math.abs(slidesTARGET - extendArmSS.getInches2())) / 2);
        telemetry.addData("Submersible Arm Position:", subArm.getPosition());
        telemetry.addData("Wrist Position1:", wrist1.getPosition());
        telemetry.addData("Wrist Position2:", wrist2.getPosition());
        telemetry.addData("Claw Position1:", claw1.getPosition());
        telemetry.addData("Claw Position2:", claw2.getPosition());
        telemetry.addData("Arm Position:", arm.getPosition());
        telemetry.addData("Rotation Position1:", rotation1.getPosition());
        telemetry.addData("Rotation Position2:", rotation2.getPosition());
        telemetry.addData("Control Hub Current", LynxUtils.getControlHubCurrent());
        telemetry.addData("Expansion Hub Current", LynxUtils.getExpansionHubCurrent());
        telemetry.addData("Servo Hub Current", LynxUtils.getServoHubCurrent());
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
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        PID controller = new PID(Math.sqrt(P), I, D);
        // motors
        extendArm1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm1"));
        extendArm2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm2"));
        // servos
        // ea
        arm1 = new CachingServo(hardwareMap.get(Servo.class, "arm1")); // 1x axon max
        arm2 = new CachingServo(hardwareMap.get(Servo.class, "arm2")); // 1x axon max
        wrist1 = new CachingServo(hardwareMap.get(Servo.class, "wrist1")); // 1x axon mini
        claw1 = new CachingServo(hardwareMap.get(Servo.class, "claw1")); // 1x axon mini
        rotation1 = new CachingServo(hardwareMap.get(Servo.class, "rotation2")); // 1x axon max
        arm = new CombinedServo(arm1, arm2); // 2x axon max
        // sa
        submersibleArm1 = new CachingServo(hardwareMap.get(Servo.class, "subArm1")); // 1x axon mini
        submersibleArm2 = new CachingServo(hardwareMap.get(Servo.class, "subArm2")); // 1x axon mini
        wrist2 = new CachingServo(hardwareMap.get(Servo.class, "wrist2")); // 1x axon mini
        claw2 = new CachingServo(hardwareMap.get(Servo.class, "claw2")); // 1x goBilda speed
        rotation2 = new CachingServo(hardwareMap.get(Servo.class, "rotation1")); // 1x axon max
        subArm = new CombinedServo(submersibleArm1, submersibleArm2); // 2x axon mini
        // directions
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // limits
        claw2.scaleRange(0, 0.3);
        wrist2.scaleRange(0.1, 0.86);
        rotation1.scaleRange(0, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.58);
        claw1.scaleRange(0, 0.43);
        subArm.scaleRange(0.25, 0.47);
        rotation2.scaleRange(0.02, 0.565);
        // extendArm
        extendArmSS = new SlidesSS(extendArm1, extendArm2, controller, K, F, CPR, INCHES_PER_REV, MainV6.eaLimitHigh, MainV6.eaLimitLow, eaCorrection, false);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        wrist1.setPosition(wristCpos1 = 1);
        wrist2.setPosition(wristCpos2 = 1);
        claw1.setPosition(clawCpos1 = 1);
        claw2.setPosition(clawCpos2 = 1);
        arm.setPosition(armCpos = 0.15);
        subArm.setPosition(subArmCpos = 1);
        rotation1.setPosition(rotationalCpos1 = 0);
        rotation2.setPosition(rotationalCpos2 = 0);
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
        // telemetry
        telemetry.addLine("BEASTKIT Team 23403!");
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

