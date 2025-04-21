/***
 * MAIN V5
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch because i hate my old code
 * based off of MainV4 but better and advancer
 * started recoding at 4/4/25  @  7:55 pm
 * finished recoding at 4/12/25 @ 4:26 pm
 * robot v5 finished building at 4/9/25
***/
package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;
import org.firstinspires.ftc.teamcode.variables.enums.PresetStates;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Sensor;
import xyz.nin1275.utils.Timer;

@Configurable
@Config("MainV5")
@TeleOp(name="Main v5", group=".ftc23403")
public class MainV5 extends LinearOpMode {
    /**
     * @TODO get limelight working in here
     * @TODO add color sensor shit
     * MAIN V5 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
    **/
    // servos
    public static double wristCpos1 = 0;
    public static double clawCpos1 = 1;
    public static double sweeperCpos = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 0.5;
    public static double armCpos = 0.23;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0.5;
    // misc
    public static boolean redSide = true;
    public static boolean debugMode = true;
    public static double wheelSpeedMax = 1;
    public static double wheelSpeedMinEA = 0.4;
    public static double wheelSpeedMinSA = 0.8;
    private double wheelSpeed = wheelSpeedMax;
    public static double rotationalSpeed = 0.2;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static double slidesTARGET = 0;
    public static double eaLimitHigh = 36;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    ElapsedTime resetTimer = new ElapsedTime();
    // states
    private static ExtendArmStates extendArmState = ExtendArmStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
    // presets
    @Config("MainV5 Presets")
    public static class presets {
        public static CustomPresets humanPlayer = new CustomPresets(
                eaLimitLow,
                -1.0,
                -1.0,
                0.0,
                -1.0,
                0.4,
                0.97,
                -1.0);
        public static CustomPresets highBasket = new CustomPresets(
                35,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.8,
                -1.0);
        public static CustomPresets lowBasket = new CustomPresets(
                16.5,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.8,
                -1.0);
        public static CustomPresets transition = new CustomPresets(
                eaLimitLow,
                1.0,
                1.0,
                0.0,
                0.9,
                0.5,
                0.18,
                0.52);
        public static CustomPresets preSpecimen = new CustomPresets(
                10,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0);
        public static CustomPresets scoreSpecimen = new CustomPresets(
                19,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0);
    }
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PIDController controller = new PIDController(Math.sqrt(P), I, D);
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower, sensor);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
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
        Servo sweeper = hardwareMap.get(Servo.class, "sweeper"); // 1x goBilda torque
        // ea
        Servo arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x goBilda speed
        // sa
        Servo submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // 1x axon
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x 25kg
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        Servo rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0, 0.8);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        submersibleArm1.scaleRange(0.45, 1);
        // turn on pwm servos
        arm.getController().pwmEnable();
        submersibleArm1.getController().pwmEnable();
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        sweeper.setDirection(Servo.Direction.REVERSE);
        // breaks
        Motors.setBrakes(List.of(leftFront, rightFront, leftRear, rightRear));
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(true, true, 255, 0, 255);
        // misc
        claw1.setPosition(clawCpos1);
        // calibration
        hardwareMap.get(IMU.class, "imu").resetYaw();
        if (Calibrate.Auto.getLastKnownPos() != null) {
            follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        } else {
            follower.setStartingPose(new Pose(0,0,0));
        }
        Calibrate.Auto.clearEverything();
        // Draw the robot on the dashboard
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        if (Calibrate.Auto.getLastKnownPos() != null) {
            poseUpdater.setStartingPose(Calibrate.Auto.getLastKnownPos());
        } else {
            poseUpdater.setStartingPose(new Pose(0,0,0));
        }
        DashboardPoseTracker dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        // reset slides 0 pos
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            extendArm1.setPower(-0.4);
            extendArm2.setPower(-0.4);
            telemetryM.addLine(true, "RESETTING 0 POS!");
            telemetryM.update();
        }
        extendArm1.setPower(0);
        extendArm2.setPower(0);
        Motors.resetEncoders(List.of(extendArm1, extendArm2));
        Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        resetTimer.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
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
                submersibleArm1.setPosition(subArmCpos);
                sweeper.setPosition(sweeperCpos);
                rotation.setPosition(rotationalCpos);
                // field side
                if (currentGamepad1.share && !previousGamepad1.share) {
                    redSide = !redSide;
                }
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
                // extendArm code
                controller.setPID(Math.sqrt(P), I, D);
                // Get current positions
                int eaTicks1 = extendArm1.getCurrentPosition();
                int eaTicks2 = extendArm2.getCurrentPosition();
                // Convert ticks to inches
                double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
                double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
                // vars
                double ff = eaCorrection ? F : 0;
                // controls
                if (gamepad2.dpad_up && eaInches1 < eaLimitHigh) {
                    double pid = controller.calculate(eaInches1, eaLimitHigh);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
                } else if (gamepad2.dpad_down && eaInches1 > eaLimitLow) {
                    double pid = controller.calculate(eaInches1, eaLimitLow);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
                } else if (Math.abs(eaInches1 - eaLimitLow) > 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                    extendArm1.setPower(ff);
                    extendArm2.setPower(ff);
                    if (extendArmState == ExtendArmStates.PRESET_REACHED) Timer.wait(500);
                    extendArmState = eaCorrection ? ExtendArmStates.FORCE_FEED_BACK : ExtendArmStates.FLOATING;
                }
                // states
                if (Math.abs(eaInches1 - eaLimitHigh) < 1 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                    extendArmState = ExtendArmStates.MAX_POS;
                } else if (Math.abs(eaInches1 - eaLimitLow) < 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET && extendArmState != ExtendArmStates.RESETTING_ZERO_POS && extendArmState != ExtendArmStates.ZERO_POS_RESET && extendArmState != ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                    extendArmState = ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (extendArmState == ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(eaInches1 - eaLimitLow) < 2) {
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
                // submersibleArm code
                if (gamepad1.dpad_up) {
                    subArmCpos = 0;
                } else if (gamepad1.dpad_down) {
                    subArmCpos = 1;
                }
                // preset code
                switch (presetState) {
                    // use correction code cuz its easier fr fr
                    case HUMAN_PLAYER:
                        slidesTARGET = MainV5.presets.humanPlayer.extendArm != -1.0 ? MainV5.presets.humanPlayer.extendArm : eaInches1;
                        subArmCpos = MainV5.presets.humanPlayer.subArm != -1.0 ? MainV5.presets.humanPlayer.subArm : subArmCpos;
                        clawCpos2 = MainV5.presets.humanPlayer.claw2 != -1.0 ? MainV5.presets.humanPlayer.claw2 : clawCpos2;
                        wristCpos2 = MainV5.presets.humanPlayer.wrist2 != -1.0 ? MainV5.presets.humanPlayer.wrist2 : wristCpos2;
                        wristCpos1 = MainV5.presets.humanPlayer.wrist1 != -1.0 ? MainV5.presets.humanPlayer.wrist1 : wristCpos1;
                        clawCpos1 = MainV5.presets.humanPlayer.claw1 != -1.0 ? MainV5.presets.humanPlayer.claw1 : clawCpos1;
                        armCpos = MainV5.presets.humanPlayer.arm != -1.0 ? MainV5.presets.humanPlayer.arm : armCpos;
                        rotationalCpos = MainV5.presets.humanPlayer.rotational != -1.0 ? MainV5.presets.humanPlayer.rotational : rotationalCpos;
                        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case HIGH_BASKET:
                        slidesTARGET = MainV5.presets.highBasket.extendArm != -1.0 ? MainV5.presets.highBasket.extendArm : eaInches1;
                        subArmCpos = MainV5.presets.highBasket.subArm != -1.0 ? MainV5.presets.highBasket.subArm : subArmCpos;
                        clawCpos2 = MainV5.presets.highBasket.claw2 != -1.0 ? MainV5.presets.highBasket.claw2 : clawCpos2;
                        wristCpos2 = MainV5.presets.highBasket.wrist2 != -1.0 ? MainV5.presets.highBasket.wrist2 : wristCpos2;
                        wristCpos1 = MainV5.presets.highBasket.wrist1 != -1.0 ? MainV5.presets.highBasket.wrist1 : wristCpos1;
                        clawCpos1 = MainV5.presets.highBasket.claw1 != -1.0 ? MainV5.presets.highBasket.claw1 : clawCpos1;
                        armCpos = MainV5.presets.highBasket.arm != -1.0 ? MainV5.presets.highBasket.arm : armCpos;
                        rotationalCpos = MainV5.presets.highBasket.rotational != -1.0 ? MainV5.presets.highBasket.rotational : rotationalCpos;
                        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case LOW_BASKET:
                        slidesTARGET = MainV5.presets.lowBasket.extendArm != -1.0 ? MainV5.presets.lowBasket.extendArm : eaInches1;
                        subArmCpos = MainV5.presets.lowBasket.subArm != -1.0 ? MainV5.presets.lowBasket.subArm : subArmCpos;
                        clawCpos2 = MainV5.presets.lowBasket.claw2 != -1.0 ? MainV5.presets.lowBasket.claw2 : clawCpos2;
                        wristCpos2 = MainV5.presets.lowBasket.wrist2 != -1.0 ? MainV5.presets.lowBasket.wrist2 : wristCpos2;
                        wristCpos1 = MainV5.presets.lowBasket.wrist1 != -1.0 ? MainV5.presets.lowBasket.wrist1 : wristCpos1;
                        clawCpos1 = MainV5.presets.lowBasket.claw1 != -1.0 ? MainV5.presets.lowBasket.claw1 : clawCpos1;
                        armCpos = MainV5.presets.lowBasket.arm != -1.0 ? MainV5.presets.lowBasket.arm : armCpos;
                        rotationalCpos = MainV5.presets.lowBasket.rotational != -1.0 ? MainV5.presets.lowBasket.rotational : rotationalCpos;
                        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case TRANSITION:
                        slidesTARGET = MainV5.presets.transition.extendArm != -1.0 ? MainV5.presets.transition.extendArm : eaInches1;
                        subArmCpos = MainV5.presets.transition.subArm != -1.0 ? MainV5.presets.transition.subArm : subArmCpos;
                        clawCpos2 = MainV5.presets.transition.claw2 != -1.0 ? MainV5.presets.transition.claw2 : clawCpos2;
                        wristCpos2 = MainV5.presets.transition.wrist2 != -1.0 ? MainV5.presets.transition.wrist2 : wristCpos2;
                        wristCpos1 = MainV5.presets.transition.wrist1 != -1.0 ? MainV5.presets.transition.wrist1 : wristCpos1;
                        clawCpos1 = MainV5.presets.transition.claw1 != -1.0 ? MainV5.presets.transition.claw1 : clawCpos1;
                        armCpos = MainV5.presets.transition.arm != -1.0 ? MainV5.presets.transition.arm : armCpos;
                        rotationalCpos = MainV5.presets.transition.rotational != -1.0 ? MainV5.presets.transition.rotational : rotationalCpos;
                        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case SCORE_SPECIMEN:
                        if (Math.abs(eaInches1 - MainV5.presets.scoreSpecimen.extendArm != -1.0 ? MainV5.presets.scoreSpecimen.extendArm : eaInches1) <= 2) {
                            presetState = PresetStates.HUMAN_PLAYER;
                        }
                        break;
                }
                /**
                 * GAMEPAD 1
                 *   X / ▢         - Grab sample using limelight
                 *   Y / Δ         - EMPTY
                 *   B / O         - Grab from Human Player Preset
                 *   A / X         - EMPTY
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
                // limelight grabbing
                if (gamepad1.x) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = 0;
                    subArmCpos = 1;
                    if (true) {
                        wristCpos2 = 0.1;
                        Timer.wait(300);
                        claw2.setPosition(1);
                    }
                    extendArmState = ExtendArmStates.MOVING_TO_PRESET;
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
                if (currentGamepad2.b && !previousGamepad2.b) {
                    switch (presetState) {
                        case NO_PRESET:
                            slidesTARGET = MainV5.presets.preSpecimen.extendArm != -1.0 ? MainV5.presets.preSpecimen.extendArm : eaInches1;
                            subArmCpos = MainV5.presets.preSpecimen.subArm != -1.0 ? MainV5.presets.preSpecimen.subArm : subArmCpos;
                            clawCpos2 = MainV5.presets.preSpecimen.claw2 != -1.0 ? MainV5.presets.preSpecimen.claw2 : clawCpos2;
                            wristCpos2 = MainV5.presets.preSpecimen.wrist2 != -1.0 ? MainV5.presets.preSpecimen.wrist2 : wristCpos2;
                            wristCpos1 = MainV5.presets.preSpecimen.wrist1 != -1.0 ? MainV5.presets.preSpecimen.wrist1 : wristCpos1;
                            clawCpos1 = MainV5.presets.preSpecimen.claw1 != -1.0 ? MainV5.presets.preSpecimen.claw1 : clawCpos1;
                            armCpos = MainV5.presets.preSpecimen.arm != -1.0 ? MainV5.presets.preSpecimen.arm : armCpos;
                            rotationalCpos = MainV5.presets.preSpecimen.rotational != -1.0 ? MainV5.presets.preSpecimen.rotational : rotationalCpos;
                            extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                            presetState = PresetStates.PRE_SPECIMEN_SCORE;
                            break;
                        case PRE_SPECIMEN_SCORE:
                            slidesTARGET = MainV5.presets.scoreSpecimen.extendArm != -1.0 ? MainV5.presets.scoreSpecimen.extendArm : eaInches1;
                            subArmCpos = MainV5.presets.scoreSpecimen.subArm != -1.0 ? MainV5.presets.scoreSpecimen.subArm : subArmCpos;
                            clawCpos2 = MainV5.presets.scoreSpecimen.claw2 != -1.0 ? MainV5.presets.scoreSpecimen.claw2 : clawCpos2;
                            wristCpos2 = MainV5.presets.scoreSpecimen.wrist2 != -1.0 ? MainV5.presets.scoreSpecimen.wrist2 : wristCpos2;
                            wristCpos1 = MainV5.presets.scoreSpecimen.wrist1 != -1.0 ? MainV5.presets.scoreSpecimen.wrist1 : wristCpos1;
                            clawCpos1 = MainV5.presets.scoreSpecimen.claw1 != -1.0 ? MainV5.presets.scoreSpecimen.claw1 : clawCpos1;
                            armCpos = MainV5.presets.scoreSpecimen.arm != -1.0 ? MainV5.presets.scoreSpecimen.arm : armCpos;
                            rotationalCpos = MainV5.presets.scoreSpecimen.rotational != -1.0 ? MainV5.presets.scoreSpecimen.rotational : rotationalCpos;
                            extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                            presetState = PresetStates.SCORE_SPECIMEN;
                            break;
                    }
                }
                // auto move arm to score when we pickup from human player
                if (armCpos == MainV5.presets.humanPlayer.arm && wristCpos1 == MainV5.presets.humanPlayer.wrist1 && clawCpos1 == 1 && moving) {
                    // specimen preset
                    slidesTARGET = MainV5.presets.preSpecimen.extendArm != -1.0 ? MainV5.presets.preSpecimen.extendArm : eaInches1;
                    subArmCpos = MainV5.presets.preSpecimen.subArm != -1.0 ? MainV5.presets.preSpecimen.subArm : subArmCpos;
                    clawCpos2 = MainV5.presets.preSpecimen.claw2 != -1.0 ? MainV5.presets.preSpecimen.claw2 : clawCpos2;
                    wristCpos2 = MainV5.presets.preSpecimen.wrist2 != -1.0 ? MainV5.presets.preSpecimen.wrist2 : wristCpos2;
                    wristCpos1 = MainV5.presets.preSpecimen.wrist1 != -1.0 ? MainV5.presets.preSpecimen.wrist1 : wristCpos1;
                    clawCpos1 = MainV5.presets.preSpecimen.claw1 != -1.0 ? MainV5.presets.preSpecimen.claw1 : clawCpos1;
                    armCpos = MainV5.presets.preSpecimen.arm != -1.0 ? MainV5.presets.preSpecimen.arm : armCpos;
                    rotationalCpos = MainV5.presets.preSpecimen.rotational != -1.0 ? MainV5.presets.preSpecimen.rotational : rotationalCpos;
                    extendArmState = ExtendArmStates.MOVING_TO_PRESET;
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
                if (gamepad1.right_trigger > 0) {
                    rotationalCpos += rotationalSpeed;
                    if (rotationalCpos > 0.7) rotationalCpos = 0.7;
                } else if (gamepad1.left_trigger > 0) {
                    rotationalCpos -= rotationalSpeed;
                    if (rotationalCpos < 0.2) rotationalCpos = 0.2;
                }
                // extendArm and subArm slowdown
                if (eaInches1 >= eaLimitHigh / 2) {
                    double ratio = (eaLimitHigh - eaInches1) / (eaLimitHigh - (eaLimitHigh / 2));
                    wheelSpeed = wheelSpeedMinEA + ratio * (wheelSpeedMax - wheelSpeedMinEA);
                } else if (subArmCpos < 0.35) {
                    double subArmRatio = subArmCpos / 0.35;  // 1 when at 0.35, 0 when at 0
                    double adjustedSpeed = wheelSpeedMinSA + subArmRatio * (wheelSpeed - wheelSpeedMinSA);
                    wheelSpeed = Math.max(wheelSpeedMinSA, Math.min(adjustedSpeed, wheelSpeed));
                } else {
                    wheelSpeed = wheelSpeedMax;
                }
                // Draw the robot on the dashboard
                poseUpdater.update();
                dashboardPoseTracker.update();
                Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
                Drawing.sendPacket();
                // telemetry
                telemetryM.addLine("BEASTKIT Team 23403!");
                telemetryM.addData(true, "extendArmState", extendArmState);
                telemetryM.addData(true, "presetState", presetState);
                telemetryM.addData(true, "PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
                telemetryM.addData(true, "target", slidesTARGET);
                telemetryM.addData(true, "eaCpos1", eaInches1);
                telemetryM.addData(true, "eaCpos2", eaInches2);
                telemetryM.addData(true, "preset error1", Math.abs(slidesTARGET - eaInches1));
                telemetryM.addData(true, "preset error2", Math.abs(slidesTARGET - eaInches2));
                telemetryM.addData(true, "preset errorAvg", (Math.abs(slidesTARGET - eaInches1) + Math.abs(slidesTARGET - eaInches2)) / 2);
                telemetryM.addData(true, "DEBUG:", "PickUp " + (Sensor.pickUpRed() ? "RED" : Sensor.pickUpBlue() ? "BLUE" : Sensor.pickUpYellow() ? "YELLOW" : "NONE"));
                telemetryM.addData(true, "DEBUG:", "Grabbed " + (Sensor.isRedGrabbed() ? "RED" : Sensor.isBlueGrabbed() ? "BLUE" : Sensor.isYellowGrabbed() ? "YELLOW" : "NONE"));
                telemetryM.addData(true, "Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
                telemetryM.addData(true, "Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
                telemetryM.addData(true, "Submersible Arm Position1:", submersibleArm1.getPosition());
                telemetryM.addData(true, "Wrist Position1:", wrist1.getPosition());
                telemetryM.addData(true, "Wrist Position2:", wrist2.getPosition());
                telemetryM.addData(true, "Claw Position1:", claw1.getPosition());
                telemetryM.addData(true, "Claw Position2:", claw2.getPosition());
                telemetryM.addData(true, "Arm Position:", arm.getPosition());
                telemetryM.addData(true, "Sweeper Position:", sweeper.getPosition());
                telemetryM.addData(true, "Rotation Position:", rotation.getPosition());
                telemetryM.addData(true, "Red side?", redSide);
                telemetryM.addData(true, "slides reset timer", resetTimer.milliseconds());
                telemetryM.addData(true, "extendArm1 Power", extendArm1.getPower());
                telemetryM.addData(true, "extendArm2 Power", extendArm2.getPower());
                telemetryM.update();
            }
        }
        if (isStopRequested()) {
            // stop code
            // turn off servos on the servo hub or spm if we get them
            arm.getController().pwmDisable();
            submersibleArm1.getController().pwmDisable();
        }
    }
}
