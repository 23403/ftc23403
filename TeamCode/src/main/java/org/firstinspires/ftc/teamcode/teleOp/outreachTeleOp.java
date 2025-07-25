/***
 * Outreach Tele-Op
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * made for outreach moments when the people need to drive the robot
 * based off of MainV6 but slower
***/
package org.firstinspires.ftc.teamcode.teleOp;

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
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.SpecModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.SubModeStates;
import org.firstinspires.ftc.teamcode.variables.presets.MainV6Presets;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.subsystems.SlidesSS;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.CombinedServo;
import xyz.nin1275.utils.Motors;

@Configurable
@Config("Outreach")
@TeleOp(name="Outreach", group=".ftc23403")
public class outreachTeleOp extends LinearOpMode {
    /**
     * Outreach TeleOp BY DAVID
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
    public static double wheelSpeedMinEA = 0.7;
    public static double wheelSpeedMinSA = 0.8;
    private double wheelSpeed = wheelSpeedMax;
    // timers
    ElapsedTime subArmThrowTimer;
    ElapsedTime transitionTimer;
    // odometry
    public static boolean odoDrive = true;
    // extend arm
    public static double slidesTARGET = 0;
    public static double eaLimitHigh = 33.6;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    // states
    SlidersStates extendArmState = SlidersStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
    RotationStates rotationState = RotationStates.MIDDLE;
    LimelightState llState = LimelightState.INIT;
    SubModeStates subStates = SubModeStates.RETURN;
    SpecModeStates specStates = SpecModeStates.GRAB;
    // config stuff
    public static int SUB_THROW_DELAY = 200;
    public static double EA_MAX_SPEED_DOWN = -0.2;
    public static boolean redSide = true;
    public static boolean debugMode = true;
    public static double wheelSpeedMax = 0.75;
    public static int TRANSITION_DELAY = 200;
    public static boolean headingLock = true;
    public static double headingTolerance = 0.1;
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PID controller = new PID(Math.sqrt(P), I, D);
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(50);
        Vision.Limelight limelight = new Vision.Limelight(limelight3A, llState, follower);
        LynxUtils.initLynx(hardwareMap);
        // gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        // motors
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));
        CachingDcMotorEx extendArm1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm1"));
        CachingDcMotorEx extendArm2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm2"));
        // servos
        // ea
        CachingServo arm1 = new CachingServo(hardwareMap.get(Servo.class, "arm1")); // 1x axon max
        CachingServo arm2 = new CachingServo(hardwareMap.get(Servo.class, "arm2")); // 1x axon max
        CachingServo wrist1 = new CachingServo(hardwareMap.get(Servo.class, "wrist1")); // 1x axon mini
        CachingServo claw1 = new CachingServo(hardwareMap.get(Servo.class, "claw1")); // 1x axon mini
        CachingServo rotation1 = new CachingServo(hardwareMap.get(Servo.class, "rotation2")); // 1x axon max
        CombinedServo arm = new CombinedServo(arm1, arm2); // 2x axon max
        // sa
        CachingServo submersibleArm1 = new CachingServo(hardwareMap.get(Servo.class, "subArm1")); // 1x axon mini
        CachingServo submersibleArm2 = new CachingServo(hardwareMap.get(Servo.class, "subArm2")); // 1x axon mini
        CachingServo wrist2 = new CachingServo(hardwareMap.get(Servo.class, "wrist2")); // 1x axon mini
        CachingServo claw2 = new CachingServo(hardwareMap.get(Servo.class, "claw2")); // 1x goBilda speed
        CachingServo rotation2 = new CachingServo(hardwareMap.get(Servo.class, "rotation1")); // 1x axon max
        CombinedServo subArm = new CombinedServo(submersibleArm1, submersibleArm2); // 2x axon mini
        // limits
        claw2.scaleRange(0, 0.3);
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
        hardwareMap.get(IMU.class, "imu").resetYaw();
        if (Calibrate.Auto.getLastKnownPos() != null) follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        else follower.setStartingPose(new Pose(7.6,63.9,0));
        Calibrate.Auto.clearEverything();
        // Draw the robot on the dashboard
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        if (Calibrate.Auto.getLastKnownPos() != null) poseUpdater.setStartingPose(Calibrate.Auto.getLastKnownPos());
        else poseUpdater.setStartingPose(new Pose(7.6,63.9,0));
        DashboardPoseTracker dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        // setup slides
        extendArmSS = new SlidesSS(extendArm1, extendArm2, controller, K, F, CPR, INCHES_PER_REV, MainV6.eaLimitHigh, MainV6.eaLimitLow, eaCorrection, false);
        // misc
        subStates = SubModeStates.RETURN;
        specStates = SpecModeStates.GRAB;
        subArmThrowTimer = new ElapsedTime();
        transitionTimer = new ElapsedTime();
        subArmThrowTimer.reset();
        transitionTimer.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            double heading = follower.getPose().getHeading();
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                limelight.update();
                llState = limelight.getState();
                extendArmState = extendArmSS.getState();
                extendArmSS.setEaCorrection(eaCorrection);
                extendArmSS.setLimits(MainV6.eaLimitHigh, MainV6.eaLimitLow);
                extendArmSS.setMaxSpeedDown(EA_MAX_SPEED_DOWN);
                limelight.setFollower(follower);
                telemetryM.setDebug(debugMode);
                boolean moving = Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0;
                // gamepad stuff
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                // servos
                wrist1.setPosition(wristCpos1);
                wrist2.setPosition(wristCpos2);
                claw1.setPosition(clawCpos1);
                claw2.setPosition(clawCpos2);
                arm.setPosition(armCpos);
                subArm.setPosition(subArmCpos);
                rotation1.setPosition(rotationalCpos1);
                rotation2.setPosition(rotationalCpos2);
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
                // heading lock
                if (Math.abs(gamepad1.right_stick_x) > 0) {
                    heading = follower.getPose().getHeading();
                } else if (Math.abs(heading - follower.getPose().getHeading()) > headingTolerance && headingLock) {
                    follower.turnTo(heading);
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
                        applyPreset(MainV6Presets.humanPlayer);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case HIGH_BASKET:
                        applyPreset(MainV6Presets.highBasket);
                        if (clawCpos1 == 0 || gamepad2.right_trigger > 0) presetState = PresetStates.HUMAN_PLAYER;
                        break;
                    case LOW_BASKET:
                        applyPreset(MainV6Presets.lowBasket);
                        if (clawCpos1 == 0 || gamepad2.right_trigger > 0) presetState = PresetStates.HUMAN_PLAYER;
                        break;
                    case TRANSITION:
                        applyPreset(MainV6Presets.transition);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case SCORE_SPECIMEN:
                        if (gamepad2.right_trigger > 0) {
                            specStates = SpecModeStates.GRAB;
                            presetState = PresetStates.HUMAN_PLAYER;
                        }
                        if (currentGamepad2.a && !previousGamepad2.a) {
                            applyPreset(MainV6Presets.preSpecimen);
                            presetState = PresetStates.PRE_SPECIMEN_SCORE;
                            specStates = SpecModeStates.PRE_SPECIMEN;
                        }
                        break;
                    case SUB_OUT:
                        applyPreset(MainV6Presets.slidesOut);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case SUB_THROW:
                        applyPreset(MainV6Presets.subThrow);
                        if (subArmThrowTimer.milliseconds() > SUB_THROW_DELAY) {
                            subStates = SubModeStates.RETURN;
                            presetState = PresetStates.HUMAN_PLAYER;
                        }
                        break;
                    case RETURN:
                        applyPreset(MainV6Presets.reTurn);
                        presetState = PresetStates.NO_PRESET;
                        break;
                }
                /**
                 * GAMEPAD 1
                 *   X / ▢         - Transition from Submersible arm to Extend arm
                 *   Y / Δ         - Limelight grabbing
                 *   B / O         - Return
                 *   A / X         - Submersible grabbing and stuff
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
                switch (subStates) {
                    case MOVE_OUT:
                        if (currentGamepad1.a && !previousGamepad1.a) {
                            clawCpos2 = 1;
                            subStates = SubModeStates.GRAB;
                        }
                        if (currentGamepad1.b && !previousGamepad1.b) {
                            presetState = PresetStates.RETURN;
                            subStates = SubModeStates.MOVE_OUT;
                        }
                        break;
                    case GRAB:
                        if (currentGamepad1.a && !previousGamepad1.a) {
                            presetState = PresetStates.RETURN;
                            subStates = SubModeStates.RETURN;
                        }
                        if (currentGamepad1.b && !previousGamepad1.b) {
                            clawCpos2 = 0;
                            subStates = SubModeStates.MOVE_OUT;
                        }
                        break;
                    case THROW:
                        if (currentGamepad1.a && !previousGamepad1.a) {
                            presetState = PresetStates.RETURN;
                            subStates = SubModeStates.RETURN;
                        }
                        break;
                    case RETURN:
                        if (currentGamepad1.a && !previousGamepad1.a) {
                            presetState = PresetStates.SUB_OUT;
                            subStates = SubModeStates.MOVE_OUT;
                        }
                        if (currentGamepad1.b && !previousGamepad1.b) {
                            subArmThrowTimer.reset();
                            presetState = PresetStates.SUB_THROW;
                            subStates = SubModeStates.THROW;
                        }
                        break;
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
                    specStates = SpecModeStates.GRAB;
                    presetState = PresetStates.HIGH_BASKET;
                }
                // low basket preset
                if (gamepad2.x) {
                    specStates = SpecModeStates.GRAB;
                    presetState = PresetStates.LOW_BASKET;
                }
                if (gamepad2.dpad_right) {
                    transition();
                    transitionTimer.reset();
                    subStates = SubModeStates.RETURN;
                    specStates = SpecModeStates.RETURN;
                }
                if (transitionTimer.milliseconds() > TRANSITION_DELAY && transitionTimer.milliseconds() < TRANSITION_DELAY + 50) transition2();
                // specimen preset
                switch (specStates) {
                    case RETURN:
                        if (gamepad2.b && !gamepad2.options) specStates = SpecModeStates.GRAB;
                        break;
                    case GRAB:
                        if (clawCpos1 == 1 && currentGamepad2.b && !previousGamepad2.b && !gamepad2.options) {
                            applyPreset(MainV6Presets.preSpecimen);
                            presetState = PresetStates.PRE_SPECIMEN_SCORE;
                            specStates = SpecModeStates.PRE_SPECIMEN;
                        }
                        if (clawCpos1 == 1 && gamepad2.a) clawCpos1 = 0;
                        break;
                    case PRE_SPECIMEN:
                        if (currentGamepad2.b && !previousGamepad2.b && !gamepad2.options) {
                            applyPreset(MainV6Presets.scoreSpecimen);
                            presetState = PresetStates.SCORE_SPECIMEN;
                            specStates = SpecModeStates.SCORE;
                        }
                        break;
                }
                // claws
                if (gamepad2.right_trigger > 0) {
                    clawCpos1 = 0;
                } else if (!(currentGamepad1.right_trigger > 0) && previousGamepad2.right_trigger > 0) {
                    clawCpos1 = 1;
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
                if (rotationalCpos1 > 0.1) rotationState = RotationStates.LEFT;
                else if (rotationalCpos1 < 0.1) rotationState = RotationStates.MIDDLE;
                if (currentGamepad1.left_trigger > 0 && !(previousGamepad1.left_trigger > 0)) {
                    rotationalCpos1 = 0;
                    rotationState = RotationStates.MIDDLE;
                } else if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)) {
                    rotationalCpos1 = 0.5;
                    rotationState = RotationStates.LEFT;
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
    }

    public void transition() {
        subArmCpos = 1;
        clawCpos2 = 0.75;
        wristCpos2 = 0.65;
        armCpos = 0.27;
        clawCpos1 = 0;
        wristCpos1 = 0.1;
    }
    public void transition2() {
        clawCpos2 = 0.9;
        rotationalCpos1 = 1;
        wristCpos2 = 1;
    }
}
