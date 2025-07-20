/***
 * MAIN V6
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch because i hate my old code again
 * based off of MainV5 but better
 * started recoding at 5/12/25  @  11:38 am
 * finished recoding at 6/13/25 @ 8:32 pm
 * robot v6 finished building at 6/08/25
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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
@Config("MainV6")
@TeleOp(name="Main v6", group=".ftc23403")
public class MainV6 extends LinearOpMode {
    /**
     * MAIN V6 BY DAVID
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
    public static double wheelSpeedMinSA = 0.6;
    private double wheelSpeed = wheelSpeedMax;
    boolean basketsTimerInit = false;
    // timers
    ElapsedTime loopTime;
    ElapsedTime subArmThrowTimer;
    ElapsedTime subWristTimer;
    ElapsedTime transitionTimer;
    ElapsedTime basketsTimer;
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
    public static int SUB_WRIST_DELAY = 200;
    public static int BASKETS_DELAY = 100;
    public static double EA_MAX_SPEED_DOWN = -0.4;
    public static boolean redSide = true;
    public static boolean debugMode = true;
    public static double wheelSpeedMax = 1;
    public static int TRANSITION_DELAY = 200;
    // heading lock
    public static boolean headingLock = false;
    public static double headingLockPos = 0;
    double headingCalc = 0;
    double headingError = 0;
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PID eaController = new PID(Math.sqrt(P), I, D);
        PID headingController = new PID(0.7, 0, 0.05);
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
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront")); // 435 rpm
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear")); // 435 rpm
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront")); // 435 rpm
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear")); // 435 rpm
        CachingDcMotorEx extendArm1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm1")); // 1150 rpm
        CachingDcMotorEx extendArm2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm2")); // 1150 rpm
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
        extendArmSS = new SlidesSS(extendArm1, extendArm2, eaController, K, F, CPR, INCHES_PER_REV, false);
        // misc
        subStates = SubModeStates.RETURN;
        specStates = SpecModeStates.GRAB;
        loopTime = new ElapsedTime();
        subArmThrowTimer = new ElapsedTime();
        transitionTimer = new ElapsedTime();
        subWristTimer = new ElapsedTime();
        basketsTimer = new ElapsedTime();
        loopTime.reset();
        subArmThrowTimer.reset();
        transitionTimer.reset();
        subWristTimer.reset();
        basketsTimer.reset();
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
                extendArmSS.updatePIDKFValues(P, I, D, K, F);
                extendArmSS.setMaxSpeedDown(EA_MAX_SPEED_DOWN);
                extendArmSS.setLimits(MainV6.eaLimitHigh, MainV6.eaLimitLow);
                limelight.setFollower(follower);
                telemetryM.setDebug(debugMode);
                boolean moving = Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0;
                double heading = follower.getPose().getHeading();
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
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, headingLock ? (Math.toDegrees(headingError) > 2 ? headingCalc : 0) : -gamepad1.right_stick_x, true);
                    follower.update();
                }
                if (!moving && !odoDrive) {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
                // heading lock
                if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                    headingLock = !headingLock;
                    if (headingLock) headingLockPos = follower.getPose().getHeading();
                }
                // calc heading error and pid
                headingCalc = headingController.calculate(heading, headingLockPos);
                headingError = Math.abs(heading - headingLockPos);
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
                    case LOW_BASKET:
                        applyPreset(presetState == PresetStates.HIGH_BASKET ? MainV6Presets.highBasket : MainV6Presets.lowBasket);
                        if (clawCpos1 == 0 || (currentGamepad2.right_trigger > 0 && !(previousGamepad2.right_trigger > 0))) {
                            basketsTimer.reset();
                            basketsTimerInit = true;
                        }
                        if (basketsTimer.milliseconds() > BASKETS_DELAY && basketsTimerInit) {
                            presetState = PresetStates.HUMAN_PLAYER;
                            basketsTimerInit = false;
                        }
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
                        if (subWristTimer.milliseconds() > SUB_WRIST_DELAY) {
                            wristCpos2 = 0;
                            presetState = PresetStates.NO_PRESET;
                        }
                        break;
                    case SUB_THROW:
                        applyPreset(MainV6Presets.subThrow);
                        if (subArmThrowTimer.milliseconds() > SUB_THROW_DELAY) {
                            subStates = SubModeStates.RETURN;
                            presetState = PresetStates.RETURN;
                        }
                        break;
                    case RETURN:
                        applyPreset(MainV6Presets.reTurn);
                        presetState = PresetStates.NO_PRESET;
                        break;
                }
                /**
                 * GAMEPAD 1
                 *   X / ▢         - N/A
                 *   Y / Δ         - N/A
                 *   B / O         - Throw block
                 *   A / X         - Submersible grabbing
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
                            if (wristCpos2 <= 0.4) {
                                wrist2.setPosition(1);
                                wristCpos2 = 1;
                            }
                            subWristTimer.reset();
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
                 *   X / ▢         - Place in Low Basket
                 *   Y / Δ         - Place in High Basket
                 *   B / O         - Score Specimen Preset
                 *   A / X         - N/A
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
                telemetryM.addData(true, "fl", leftFront.getCurrent(CurrentUnit.AMPS));
                telemetryM.addData(true, "bl", leftRear.getCurrent(CurrentUnit.AMPS));
                telemetryM.addData(true, "fr", rightFront.getCurrent(CurrentUnit.AMPS));
                telemetryM.addData(true, "br", rightRear.getCurrent(CurrentUnit.AMPS));
                telemetryM.addData(true, "ea1", extendArm1.getCurrent(CurrentUnit.AMPS));
                telemetryM.addData(true, "ea2", extendArm2.getCurrent(CurrentUnit.AMPS));
                telemetryM.addData(true, "extendArmState", extendArmState);
                telemetryM.addData(true, "presetState", presetState);
                telemetryM.addData(true, "rotationState", rotationState);
                telemetryM.addData(true, "submersible state", subStates);
                telemetryM.addData(true, "specimen state", specStates);
                telemetryM.addData(true, "llState", llState);
                telemetryM.addData(true, "heading", Math.toDegrees(heading));
                telemetryM.addData(true, "heading lock", headingLock);
                telemetryM.addData(true, "heading lock pos", Math.toDegrees(headingLockPos));
                telemetryM.addData(true, "heading calc", headingCalc);
                telemetryM.addData(true, "heading error", Math.toDegrees(headingError));
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
    }
    // transition functions
    public void transition() {
        subArmCpos = 1;
        rotationalCpos1 = 0;
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
