/***
 * MAIN V5
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch because i hate my old code
 * based off of MainV4 but better and advancer
 * started recoding at 4/4/25  @  7:55 pm
 * finished recoding at 4/12/25 @ 4:26 pm
 * robot v5 finished building at 4/9/25
***/
package org.firstinspires.ftc.teamcode.teleOp.old;

import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.K;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LimelightState;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.enums.PresetStates;
import org.firstinspires.ftc.teamcode.variables.enums.RotationStates;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.subsystems.SlidesSS;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;
import xyz.nin1275.utils.CombinedServo;

@Disabled
@Configurable
@Config("MainV5")
@TeleOp(name="Main v5", group=".ftc23403")
public class MainV5 extends LinearOpMode {
    /**
     * @TODO forget the color sensor just make this perfect for mayors
     * MAIN V5 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
    **/
    // servos
    public static double wristCpos1 = 1;
    public static double clawCpos1 = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.15;
    public static double subArmCpos = 1;
    public static double rotationalCpos1 = 0;
    public static double rotationalCpos2 = 0;
    // misc
    public static boolean redSide = true;
    public static boolean debugMode = false;
    public static double wheelSpeedMax = 1;
    public static double wheelSpeedMinEA = 0.7;
    public static double wheelSpeedMinSA = 0.8;
    private double wheelSpeed = wheelSpeedMax;
    public static double hangPower = 0.8;
    ElapsedTime loopTime;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static double slidesTARGET = 0;
    public static double eaLimitHigh = 33.6;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    public static double EA_MAX_SPEED_DOWN = -0.4;
    // states
    SlidersStates extendArmState = SlidersStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
    private static RotationStates rotationState = RotationStates.MIDDLE;
    LimelightState llState = LimelightState.INIT;
    // presets
    @Config("MainV5 Presets")
    public static class presets {
        public static CustomPresets humanPlayer = new CustomPresets(
                eaLimitLow,
                -1.0,
                -1.0,
                0.0,
                -1.0,
                0.42,
                0.96,
                -1.0,
                0.0);
        public static CustomPresets highBasket = new CustomPresets(
                27,
                -1.0,
                0.0,
                1.0,
                -1.0,
                0.4,
                0.8,
                -1.0,
                0.0);
        public static CustomPresets lowBasket = new CustomPresets(
                16.5,
                -1.0,
                0.0,
                1.0,
                -1.0,
                0.6,
                0.8,
                -1.0,
                0.0);
        public static CustomPresets transition = new CustomPresets(
                eaLimitLow,
                1.0,
                1.0,
                0.0,
                1.0,
                0.6,
                0.15,
                0.0,
                0.0);
        public static CustomPresets preSpecimen = new CustomPresets(
                7,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0,
                -1.0);
        public static CustomPresets scoreSpecimen = new CustomPresets(
                20,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0,
                -1.0);
        public static CustomPresets preHang = new CustomPresets(
                33.6,
                1.0,
                1.0,
                1.0,
                1.0,
                0.5,
                0.18,
                0.52,
                0.0);
        public static CustomPresets hang = new CustomPresets(
                eaLimitLow,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0);
    }
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PID controller = new PID(Math.sqrt(P), I, D);
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Vision.Limelight limelight = new Vision.Limelight(limelight3A, llState, follower);
        LynxUtils.initLynx(hardwareMap);
        // gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        // motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        // ea
        Servo arm1 = hardwareMap.get(Servo.class, "arm1"); // 1x axon max
        Servo arm2 = hardwareMap.get(Servo.class, "arm2"); // 1x axon max
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon mini
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x axon mini
        Servo rotation1 = hardwareMap.get(Servo.class, "rotation2"); // 1x axon max
        CombinedServo arm = new CombinedServo(arm1, arm2); // 2x axon max
        // sa
        Servo submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // 1x axon max
        Servo submersibleArm2 = hardwareMap.get(Servo.class, "subArm2"); // 1x 25kg
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x axon mini
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x axon mini
        Servo rotation2 = hardwareMap.get(Servo.class, "rotation1"); // 1x axon max
        CombinedServo subArm = new CombinedServo(submersibleArm1, submersibleArm2); // 1x axon max : 1x 25kg
        // limits
        claw2.scaleRange(0, 0.27);
        wrist2.scaleRange(0.1, 0.86);
        rotation1.scaleRange(0, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.58);
        claw1.scaleRange(0, 0.43);
        subArm.scaleRange(0.25, 0.47);
        rotation2.scaleRange(0.02, 0.565);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        Motors.setBrakes(leftFront, rightFront, leftRear, rightRear);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(true, true, 255, 0, 255);
        // starting pos
        wrist1.setPosition(1);
        wrist2.setPosition(1);
        claw1.setPosition(1);
        claw2.setPosition(1);
        arm.setPosition(0.15);
        subArm.setPosition(1);
        rotation1.setPosition(0);
        rotation2.setPosition(0);
        wristCpos1 = 1;
        clawCpos1 = 1;
        wristCpos2 = 1;
        clawCpos2 = 1;
        armCpos = 0.15;
        subArmCpos = 1;
        rotationalCpos2 = 0;
        rotationalCpos1 = 0;
        // calibration
        hardwareMap.get(IMU.class, "imu").resetYaw();
        if (Calibrate.Auto.getLastKnownPos() != null) follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        else follower.setStartingPose(new Pose(9,63.4,0));
        Calibrate.Auto.clearEverything();
        // Draw the robot on the dashboard
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        if (Calibrate.Auto.getLastKnownPos() != null) poseUpdater.setStartingPose(Calibrate.Auto.getLastKnownPos());
        else poseUpdater.setStartingPose(new Pose(9,63.4,0));
        DashboardPoseTracker dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        // setup slides
        extendArmSS = new SlidesSS(extendArm1, extendArm2, controller, K, F, CPR, INCHES_PER_REV, MainV5.eaLimitHigh, MainV5.eaLimitLow, eaCorrection, false);
        // misc
        loopTime = new ElapsedTime();
        loopTime.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                limelight.update();
                llState = limelight.getState();
                extendArmState = extendArmSS.getState();
                extendArmSS.setEaCorrection(eaCorrection);
                extendArmSS.setLimits(MainV5.eaLimitHigh, MainV5.eaLimitLow);
                extendArmSS.setMaxSpeedDown(EA_MAX_SPEED_DOWN);
                limelight.setFollower(follower);
                telemetryM.setDebug(debugMode);
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                // gamepad stuff
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                // servos
                if (Math.abs(wrist1.getPosition() - wristCpos1) > 0.02) wrist1.setPosition(wristCpos1);
                if (Math.abs(wrist2.getPosition() - wristCpos2) > 0.02) wrist2.setPosition(wristCpos2);
                if (Math.abs(claw1.getPosition() - clawCpos1) > 0.02) claw1.setPosition(clawCpos1);
                if (Math.abs(claw2.getPosition() - clawCpos2) > 0.02) claw2.setPosition(clawCpos2);
                if (Math.abs(arm.getPosition() - armCpos) > 0.02) arm.setPosition(armCpos);
                if (Math.abs(subArm.getPosition() - subArmCpos) > 0.02) subArm.setPosition(subArmCpos);
                if (Math.abs(rotation1.getPosition() - rotationalCpos1) > 0.02) rotation1.setPosition(rotationalCpos1);
                if (Math.abs(rotation2.getPosition() - rotationalCpos2) > 0.02) rotation2.setPosition(rotationalCpos2);
                // field side
                if ((currentGamepad1.share && !previousGamepad1.share) || (currentGamepad2.share && !previousGamepad2.share)) redSide = !redSide;
                // toggle debug
                if ((currentGamepad1.options && !previousGamepad1.options) || (currentGamepad2.options && !previousGamepad2.options)) debugMode = !debugMode;
                // movements
                if (!odoDrive) {
                    // drive
                    double forward = -gamepad1.left_stick_y; // forward
                    double strafe = gamepad1.left_stick_x; // strafe
                    double turn = gamepad1.right_stick_x;  // rotation
                    // formula
                    double leftFrontPower = (forward + strafe + turn) * wheelSpeed;
                    double leftBackPower = (forward - strafe + turn) * wheelSpeed;
                    double rightFrontPower = (forward - strafe - turn) * wheelSpeed;
                    double rightBackPower = (forward + strafe - turn) * wheelSpeed;
                    // power
                    leftFront.setPower(leftFrontPower);
                    leftRear.setPower(leftBackPower);
                    rightFront.setPower(rightFrontPower);
                    rightRear.setPower(rightBackPower);
                } else {
                    follower.setMaxPower(wheelSpeed);
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                    follower.update();
                }
                if (!moving && !odoDrive) {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
                // extendArm code
                extendArmSS.update(gamepad2.dpad_up, gamepad2.dpad_down);
                // submersibleArm code
                if (gamepad1.dpad_up) {
                    subArmCpos = 0;
                } else if (gamepad1.dpad_down) {
                    subArmCpos = 1;
                }
                // preset code
                switch (presetState) {
                    case HUMAN_PLAYER:
                        applyPreset(MainV5.presets.humanPlayer);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case HIGH_BASKET:
                        applyPreset(MainV5.presets.highBasket);
                        if (clawCpos1 == 0 || gamepad2.left_trigger > 0) presetState = PresetStates.TRANSITION;
                        break;
                    case LOW_BASKET:
                        applyPreset(MainV5.presets.lowBasket);
                        if (clawCpos1 == 0 || gamepad2.left_trigger > 0) presetState = PresetStates.TRANSITION;
                        break;
                    case TRANSITION:
                        applyPreset(MainV5.presets.transition);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case SCORE_SPECIMEN:
                        double scoreTarget = MainV5.presets.scoreSpecimen.extendArm != -1.0 ? MainV5.presets.scoreSpecimen.extendArm : extendArmSS.getInches1();
                        if (Math.abs(extendArmSS.getInches1() - scoreTarget) <= 2) presetState = PresetStates.HUMAN_PLAYER;
                        break;
                }
                /**
                 * GAMEPAD 1
                 *   X / ▢         - Transition from Submersible arm to Extend arm
                 *   Y / Δ         - L2 hang
                 *   B / O         - Grab from Human Player Preset
                 *   A / X         - Grab using Limelight
                 *
                 *                    ________
                 *                   / ______ \\
                 *     ------------.-'   _  '-..+
                 *              /   _  ( Y )  _  \\
                 *             |  ( X )  _  ( B ) |
                 *        ___  '.      ( A )     /|
                 *      .'    '.    '-._____.-'  .'
                 *     |       |                 |
                 *      '.___.' '.               |
                 *               '.             /
                 *                \\.          .'
                 *                  \\________/
                */
                // humanPlayer preset
                if (gamepad1.b) {
                    presetState = PresetStates.HUMAN_PLAYER;
                }
                // transition preset
                if (gamepad1.x) {
                    presetState = PresetStates.TRANSITION;
                }
                // limelight
                if (currentGamepad1.a && !previousGamepad1.a && !gamepad1.options) {
                    limelight.setState(LimelightState.MOVING_TO_SAMPLE);
                    // clawCpos2 = 0;
                    // wristCpos2 = 0.5;
                    subArmCpos = limelight.getSubmersible();
                    rotationalCpos1 = limelight.getRotation();
                    limelight.strafe();
                    Timer.wait(200);
                    // wristCpos2 = 0;
                    // clawCpos2 = 1;
                    limelight.setState(LimelightState.SAMPLE_REACHED);
                    // presetState = PresetStates.TRANSITION;
                }
                /**
                 * GAMEPAD 2
                 *   X / ▢         - Transition from Submersible arm to Extend arm
                 *   Y / Δ         - Place in High Basket
                 *   B / O         - Score Specimen Preset
                 *   A / X         - Place in Low basket
                 *
                 *                    ________
                 *                   / ______ \\
                 *     ------------.-'   _  '-..+
                 *              /   _  ( Y )  _  \\
                 *             |  ( X )  _  ( B ) |
                 *        ___  '.      ( A )     /|
                 *      .'    '.    '-._____.-'  .'
                 *     |       |                 |
                 *      '.___.' '.               |
                 *               '.             /
                 *                \\.          .'
                 *                  \\________/
                */
                // high basket preset
                if (gamepad2.y) {
                    presetState = PresetStates.HIGH_BASKET;
                }
                // low basket preset
                if (gamepad2.a) {
                    presetState = PresetStates.LOW_BASKET;
                }
                // transition preset
                if (gamepad2.x) {
                    presetState = PresetStates.TRANSITION;
                }
                // specimen preset
                if (currentGamepad2.b && !previousGamepad2.b && !gamepad2.options) {
                    if (presetState == PresetStates.NO_PRESET) {
                        applyPreset(MainV5.presets.preSpecimen);
                        presetState = PresetStates.PRE_SPECIMEN_SCORE;
                    } else if (presetState == PresetStates.PRE_SPECIMEN_SCORE) {
                        applyPreset(MainV5.presets.scoreSpecimen);
                        presetState = PresetStates.SCORE_SPECIMEN;
                    }
                }
                // auto move arm to score when we pickup from human player
                if (armCpos == MainV5.presets.humanPlayer.arm && wristCpos1 == MainV5.presets.humanPlayer.wrist1 && clawCpos1 == 1 && moving) {
                    applyPreset(MainV5.presets.preSpecimen);
                    presetState = PresetStates.PRE_SPECIMEN_SCORE;
                }
                // claws
                if (gamepad2.left_trigger > 0) {
                    clawCpos1 = 0;
                } else if (gamepad2.right_trigger > 0) {
                    clawCpos1 = 1;
                }
                if (gamepad1.left_bumper) {
                    clawCpos2 = 0;
                } else if (gamepad1.right_bumper) {
                    clawCpos2 = 1;
                }
                // wrist
                if (gamepad1.dpad_right) {
                    wristCpos2 = 0;
                } else if (gamepad1.dpad_left) {
                    wristCpos2 = 1;
                }
                if (gamepad2.dpad_left) {
                    wristCpos1 = 0;
                } else if(gamepad2.dpad_right) {
                    wristCpos1 = 0.6;
                }
                // arm
                if (gamepad2.right_stick_y > 0.3) {
                    armCpos = 0.23;
                    wristCpos1 = 0.6;
                } else if (gamepad2.right_stick_y < -0.3) {
                    armCpos = 0.96;
                    wristCpos1 = 0.42;
                }
                // rotate
                if (rotationalCpos1 > 0.51) rotationState = RotationStates.RIGHT;
                if (rotationalCpos1 < 0.49) rotationState = RotationStates.LEFT;
                if (rotationalCpos1 > 0.49 && rotationalCpos1 < 0.51) rotationState = RotationStates.MIDDLE;
                if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)) {
                    if (rotationState == RotationStates.MIDDLE) {
                        rotationalCpos1 = 0.7;
                        rotationState = RotationStates.RIGHT;
                    } else if (rotationState == RotationStates.LEFT) {
                        rotationalCpos1 = 0.5;
                        rotationState = RotationStates.MIDDLE;
                    }
                } else if (currentGamepad1.left_trigger > 0 && !(previousGamepad1.left_trigger > 0)) {
                    if (rotationState == RotationStates.MIDDLE) {
                        rotationalCpos1 = 0.2;
                        rotationState = RotationStates.LEFT;
                    } else if (rotationState == RotationStates.RIGHT) {
                        rotationalCpos1 = 0.5;
                        rotationState = RotationStates.MIDDLE;
                    }
                }
                // extendArm and subArm slowdown
                if (extendArmSS.getInches1() >= eaLimitHigh / 2) {
                    double ratio = (eaLimitHigh - extendArmSS.getInches1()) / (eaLimitHigh - (eaLimitHigh / 2));
                    wheelSpeed = wheelSpeedMinEA + ratio * (wheelSpeedMax - wheelSpeedMinEA);
                } else if (subArmCpos < 0.35) {
                    double subArmRatio = subArmCpos / 0.35;  // 1 when at 0.35, 0 when at 0
                    double adjustedSpeed = wheelSpeedMinSA + subArmRatio * (wheelSpeed - wheelSpeedMinSA);
                    wheelSpeed = Math.max(wheelSpeedMinSA, Math.min(adjustedSpeed, wheelSpeed));
                } else wheelSpeed = wheelSpeedMax;
                // Draw the robot on the dashboard
                if (debugMode) {
                    poseUpdater.update();
                    dashboardPoseTracker.update();
                    Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                    Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
                    Drawing.sendPacket();
                }
                // telemetry
                telemetryM.addLine("BEASTKIT Team 23403!");
                telemetryM.addData(true, "extendArmState", extendArmState);
                telemetryM.addData(true, "presetState", presetState);
                telemetryM.addData(true, "rotationState", rotationState);
                telemetryM.addData(true, "llState", llState);
                telemetryM.addData(true, "PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
                telemetryM.addData(true, "target", slidesTARGET);
                telemetryM.addData(true, "eaCpos1", extendArmSS.getInches1());
                telemetryM.addData(true, "eaCpos2", extendArmSS.getInches2());
                telemetryM.addData(true, "preset error1", Math.abs(slidesTARGET - extendArmSS.getInches1()));
                telemetryM.addData(true, "preset error2", Math.abs(slidesTARGET - extendArmSS.getInches2()));
                telemetryM.addData(true, "preset errorAvg", (Math.abs(slidesTARGET - extendArmSS.getInches1()) + Math.abs(slidesTARGET - extendArmSS.getInches2())) / 2);
                telemetryM.addData(true, "Submersible Arm Position:", subArm.getPosition());
                telemetryM.addData(true, "Wrist Position1:", wrist1.getPosition());
                telemetryM.addData(true, "Wrist Position2:", wrist2.getPosition());
                telemetryM.addData(true, "Claw Position1:", claw1.getPosition());
                telemetryM.addData(true, "Claw Position2:", claw2.getPosition());
                telemetryM.addData(true, "Arm Position:", arm.getPosition());
                telemetryM.addData(true, "Rotation Position1:", rotation1.getPosition());
                telemetryM.addData(true, "Rotation Position2:", rotation2.getPosition());
                telemetryM.addData(true, "Red side?", redSide);
                telemetryM.addData(true, "slides reset timer", extendArmSS.getResetTimer().milliseconds());
                telemetryM.addData(true, "extendArm1 Power", extendArm1.getPower());
                telemetryM.addData(true, "extendArm2 Power", extendArm2.getPower());
                telemetryM.addData(true, "Loop Times", loopTime.milliseconds());
                telemetryM.update();
                loopTime.reset();
            }
        }
        if (isStopRequested() || !isStarted()) {
            // stop code
            LynxUtils.setLynxColor(0, 255, 0);
        }
    }
    // preset controls
    public void applyPreset(CustomPresets preset) {
        // use correction code cuz its easier fr fr
        slidesTARGET = preset.extendArm != -1.0 ? preset.extendArm : extendArmSS.getInches1();
        subArmCpos = preset.subArm != -1.0 ? preset.subArm : subArmCpos;
        clawCpos2 = preset.claw2 != -1.0 ? preset.claw2 : clawCpos2;
        wristCpos2 = preset.wrist2 != -1.0 ? preset.wrist2 : wristCpos2;
        wristCpos1 = preset.wrist1 != -1.0 ? preset.wrist1 : wristCpos1;
        clawCpos1 = preset.claw1 != -1.0 ? preset.claw1 : clawCpos1;
        armCpos = preset.arm != -1.0 ? preset.arm : armCpos;
        rotationalCpos2 = preset.rotational2 != -1.0 ? preset.rotational2 : rotationalCpos2;
        rotationalCpos1 = preset.rotational1 != -1.0 ? preset.rotational1 : rotationalCpos1;
        extendArmSS.moveTo(slidesTARGET);
        presetState = PresetStates.NO_PRESET;
    }
}
