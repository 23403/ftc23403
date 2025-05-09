/***
 * Outreach Tele-Op
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * made for outreach moments when the people need to drive the robot
 * based off of MainV5 but simpiler
 * started at 4/10/25  @  11:29 am
***/
package org.firstinspires.ftc.teamcode.teleOp;

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
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

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

@Configurable
@Config("Outreach")
@TeleOp(name="Outreach", group=".ftc23403")
public class outreachTeleOp extends LinearOpMode {
    /**
     * Outreach Tele-Op BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
    **/
    // servos
    public static double wristCpos1 = 0.5;
    public static double clawCpos1 = 1;
    public static double swiperCpos = 0;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.18;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0.52;
    // misc
    public static boolean redSide = true;
    public static boolean debugMode = false;
    public static double wheelSpeedMax = 1;
    public static double wheelSpeedMinEA = 0.4;
    public static double wheelSpeedMinSA = 0.8;
    private double wheelSpeed = wheelSpeedMax;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static double slidesTARGET = 0;
    public static double eaLimitHigh = 33.6;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    // states
    SlidersStates extendArmState = SlidersStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
    private static RotationStates rotationState = RotationStates.MIDDLE;
    LimelightState llState = LimelightState.INIT;
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PID controller = new PID(Math.sqrt(P), I, D);
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower, sensor);
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
        Servo swiper = hardwareMap.get(Servo.class, "swiper"); // 1x goBilda speed
        // ea
        Servo arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x goBilda speed
        // sa
        Servo submersibleArm = hardwareMap.get(Servo.class, "subArm"); // 2x axon
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x axon
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        Servo rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0.05, 0.88);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        submersibleArm.scaleRange(0.45, 1);
        swiper.scaleRange(0.3, 0.87);
        // turn on pwm servos
        arm.getController().pwmEnable();
        submersibleArm.getController().pwmEnable();
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        swiper.setDirection(Servo.Direction.REVERSE);
        // breaks
        Motors.setBrakes(leftFront, rightFront, leftRear, rightRear);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(true, true, 255, 0, 255);
        // misc
        claw1.setPosition(clawCpos1);
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
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
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
                limelight.setFollower(follower);
                telemetryM.setDebug(debugMode);
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                // gamepad stuff
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                // servos
                if (Math.abs(wrist1.getPosition() - wristCpos1) > 0.01) wrist1.setPosition(wristCpos1);
                if (Math.abs(wrist2.getPosition() - wristCpos2) > 0.01) wrist2.setPosition(wristCpos2);
                if (Math.abs(claw1.getPosition() - clawCpos1) > 0.01) claw1.setPosition(clawCpos1);
                if (Math.abs(claw2.getPosition() - clawCpos2) > 0.01) claw2.setPosition(clawCpos2);
                if (Math.abs(arm.getPosition() - armCpos) > 0.01) arm.setPosition(armCpos);
                if (Math.abs(submersibleArm.getPosition() - subArmCpos) > 0.01) submersibleArm.setPosition(subArmCpos);
                if (Math.abs(swiper.getPosition() - swiperCpos) > 0.01) swiper.setPosition(swiperCpos);
                if (Math.abs(rotation.getPosition() - rotationalCpos) > 0.01) rotation.setPosition(rotationalCpos);
                // field side
                if (currentGamepad1.share && !previousGamepad1.share) redSide = !redSide;
                // toggle debug
                if (currentGamepad1.options && !previousGamepad1.options) debugMode = !debugMode;
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
                if (presetState != PresetStates.L2_HANG) extendArmSS.update(gamepad2.dpad_up, gamepad2.dpad_down);
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
                        if (clawCpos1 == 0) presetState = PresetStates.TRANSITION;
                        break;
                    case LOW_BASKET:
                        applyPreset(MainV5.presets.lowBasket);
                        if (clawCpos1 == 0) presetState = PresetStates.TRANSITION;
                        break;
                    case TRANSITION:
                        applyPreset(MainV5.presets.transition);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case SCORE_SPECIMEN:
                        double scoreTarget = MainV5.presets.scoreSpecimen.extendArm != -1.0 ? MainV5.presets.scoreSpecimen.extendArm : extendArmSS.getInches1();
                        if (Math.abs(extendArmSS.getInches1() - scoreTarget) <= 2) presetState = PresetStates.HUMAN_PLAYER;
                        break;
                    case L2_HANG:
                        double target = MainV5.presets.hang.extendArm != -1.0 ? MainV5.presets.hang.extendArm : extendArmSS.getInches1();
                        if (Math.abs(extendArmSS.getInches1() - target) <= 2) {
                            extendArm1.setPower(-0.8);
                            extendArm2.setPower(-0.8);
                        }
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
                // l2 hang
                if (currentGamepad1.y && !previousGamepad1.y) {
                    if (presetState == PresetStates.NO_PRESET) {
                        applyPreset(MainV5.presets.preHang);
                        presetState = PresetStates.PRE_L2_HANG;
                    } else if (presetState == PresetStates.PRE_L2_HANG) {
                        applyPreset(MainV5.presets.hang);
                        presetState = PresetStates.L2_HANG;
                    }
                }
                // limelight
                if (currentGamepad1.a && !previousGamepad1.a && !gamepad1.options) {
                    limelight.setState(LimelightState.MOVING_TO_SAMPLE);
                    // clawCpos2 = 0;
                    // wristCpos2 = 0.5;
                    subArmCpos = limelight.getSubmersible();
                    rotationalCpos = limelight.getRotation();
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
                if (currentGamepad2.b && !previousGamepad2.b && !gamepad1.options) {
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
                if (gamepad2.left_trigger > 0 || gamepad2.right_bumper) {
                    clawCpos1 = 0;
                } else if (gamepad2.right_trigger > 0 || gamepad2.left_bumper) {
                    clawCpos1 = 1;
                }
                if (gamepad1.left_bumper) {
                    clawCpos2 = 0;
                } else if (gamepad1.right_bumper) {
                    clawCpos2 = 1;
                }
                // rotate
                if (rotationalCpos > 0.51) rotationState = RotationStates.RIGHT;
                if (rotationalCpos < 0.49) rotationState = RotationStates.LEFT;
                if (rotationalCpos > 0.49 && rotationalCpos < 0.51) rotationState = RotationStates.MIDDLE;
                if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)) {
                    if (rotationState == RotationStates.MIDDLE) {
                        rotationalCpos = 0.7;
                        rotationState = RotationStates.RIGHT;
                    } else if (rotationState == RotationStates.LEFT) {
                        rotationalCpos = 0.5;
                        rotationState = RotationStates.MIDDLE;
                    }
                } else if (currentGamepad1.left_trigger > 0 && !(previousGamepad1.left_trigger > 0)) {
                    if (rotationState == RotationStates.MIDDLE) {
                        rotationalCpos = 0.2;
                        rotationState = RotationStates.LEFT;
                    } else if (rotationState == RotationStates.RIGHT) {
                        rotationalCpos = 0.5;
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
                telemetryM.update();
            }
        }
        if (isStopRequested()) {
            // stop code
            // turn off servos on the servo hub or spm if we get them
            arm.getController().pwmDisable();
            submersibleArm.getController().pwmDisable();
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
        rotationalCpos = preset.rotational != -1.0 ? preset.rotational : rotationalCpos;
        extendArmSS.moveTo(slidesTARGET);
    }
}
