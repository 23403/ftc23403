package org.firstinspires.ftc.teamcode.auto.tools;

import static org.firstinspires.ftc.teamcode.teleOp.MainV5.eaLimitHigh;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import xyz.nin1275.utils.Motors;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides;
import org.firstinspires.ftc.teamcode.utils.CustomPedroPathing;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.extendArmStates;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Test out points using the dashboard without having to upload code.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.2, 3/29/25
 */

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
    // Get current positions
    int eaTicks1 = 0;
    int eaTicks2 = 0;
    // Convert ticks to inches
    double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
    double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
    // start pos
    public static double startPosX = 9;
    public static double startPosY = 63.4;
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
            -1.0);

    /** line positions */
    private static final Pose startPos = new Pose(startPosX, startPosY, Math.toRadians(startPosRotation)); // start Pos
    /** different modes */
    private Path point;

    /** points */
    /* line1 */
    public static CustomPedroPathing.beizerLine pointPoints = new CustomPedroPathing.beizerLine(
            38.4,
            84,
            startPos.getX(),
            startPos.getY(),
            startPos.getHeading(),
            0
    );
    /** create paths */
    public void buildPaths() {
        /* line1 */
        point = new Path(
                new BezierLine(
                        pointPoints.getStartPoint(),
                        pointPoints.getEndPoint()
                ));
        point.setLinearHeadingInterpolation(Math.toRadians(pointPoints.getStartHeading()), Math.toRadians(pointPoints.getEndHeading()));
    }
    /** movements **/
    public void autonomousPathUpdate() {
        if (pathState == 0) { /* line1 */
            if (!pointStarted) {
                // movements
                extendArmMove(movement.extendArm != -1.0 ? movement.extendArm : eaInches1);
                submersibleArm(movement.subArm != -1.0 ? movement.subArm : subArmCpos);
                claw2(movement.claw2 != -1.0 ? movement.claw2 : clawCpos2);
                wrist2(movement.wrist2 != -1.0 ? movement.wrist2 : wristCpos2);
                wrist1(movement.wrist1 != -1.0 ? movement.wrist1 : wristCpos1);
                claw1(movement.claw1 != -1.0 ? movement.claw1 : clawCpos1);
                arm(movement.arm != -1.0 ? movement.arm : armCpos);
                rotation(movement.rotational != -1.0 ? movement.rotational : rotationalCpos);
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
        controller.setPID(Math.sqrt(P), I, D);
        // Get current positions
        eaTicks1 = extendArm1.getCurrentPosition();
        eaTicks2 = extendArm2.getCurrentPosition();
        // Convert ticks to inches
        eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
        eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
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
        telemetry.addData("currentState", extendArmState);
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
        // extendArm
        Motors.resetEncoders(List.of(extendArm1, extendArm2));
        Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        resetTimer.reset();
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

