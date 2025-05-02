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

import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;
import org.firstinspires.ftc.teamcode.variables.enums.PresetStates;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
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
    public static double wristCpos1 = 0;
    public static double clawCpos1 = 1;
    public static double swiperCpos = 0;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
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
    public static double eaLimitHigh = 33.6;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    public static int eaTicks1 = 0;
    public static int eaTicks2 = 0;
    public static double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
    public static double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
    ElapsedTime resetTimer = new ElapsedTime();
    // states
    private static ExtendArmStates extendArmState = ExtendArmStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
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
        Servo swiper = hardwareMap.get(Servo.class, "swiper"); // 1x goBilda speed
        // ea
        Servo arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x goBilda speed
        // sa
        Servo submersibleArm1 = hardwareMap.get(Servo.class, "subArm"); // 1x axon
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
        submersibleArm1.scaleRange(0.45, 1);
        swiper.scaleRange(0.3, 0.87);
        // turn on pwm servos
        arm.getController().pwmEnable();
        submersibleArm1.getController().pwmEnable();
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
        }
        extendArm1.setPower(0);
        extendArm2.setPower(0);
        Motors.resetEncoders(extendArm1, extendArm2);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
        resetTimer.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                telemetryM.setDebug(debugMode);
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
                swiper.setPosition(swiperCpos);
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
                eaTicks1 = extendArm1.getCurrentPosition();
                eaTicks2 = extendArm2.getCurrentPosition();
                // Convert ticks to inches
                eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
                eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
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
                    presetState = PresetStates.NO_PRESET;
                } else if (gamepad2.dpad_down && eaInches1 > eaLimitLow) {
                    double pid = controller.calculate(eaInches1, eaLimitLow);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
                    presetState = PresetStates.NO_PRESET;
                } else if (Math.abs(eaInches1 - eaLimitLow) > 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET && presetState != PresetStates.L2_HANG) {
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
                        Motors.resetEncoders(extendArm1, extendArm2);
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
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
                    case HUMAN_PLAYER:
                        applyPreset(MainV5.presets.humanPlayer);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case HIGH_BASKET:
                        applyPreset(MainV5.presets.highBasket);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case LOW_BASKET:
                        applyPreset(MainV5.presets.lowBasket);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case TRANSITION:
                        applyPreset(MainV5.presets.transition);
                        presetState = PresetStates.NO_PRESET;
                        break;
                    case SCORE_SPECIMEN:
                        double scoreTarget = MainV5.presets.scoreSpecimen.extendArm != -1.0 ? MainV5.presets.scoreSpecimen.extendArm : eaInches1;
                        if (Math.abs(eaInches1 - scoreTarget) <= 2) {
                            presetState = PresetStates.HUMAN_PLAYER;
                        }
                        break;
                    case L2_HANG:
                        double target = MainV5.presets.hang.extendArm != -1.0 ? MainV5.presets.hang.extendArm : eaInches1;
                        if (Math.abs(eaInches1 - target) <= 2) {
                            extendArm1.setPower(-0.5);
                            extendArm2.setPower(-0.5);
                        }
                        break;
                }

                /**
                 * GAMEPAD 1
                 *   X / ▢         - Transition from Submersible arm to Extend arm
                 *   Y / Δ         - L2 hang
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
                // wrist
                if (gamepad1.dpad_right) {
                    wristCpos2 = 0;
                } else if (gamepad1.dpad_left) {
                    wristCpos2 = 1;
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
    // preset controls
    public void applyPreset(CustomPresets preset) {
        // use correction code cuz its easier fr fr
        slidesTARGET = preset.extendArm != -1.0 ? preset.extendArm : eaInches1;
        subArmCpos = preset.subArm != -1.0 ? preset.subArm : subArmCpos;
        clawCpos2 = preset.claw2 != -1.0 ? preset.claw2 : clawCpos2;
        wristCpos2 = preset.wrist2 != -1.0 ? preset.wrist2 : wristCpos2;
        wristCpos1 = preset.wrist1 != -1.0 ? preset.wrist1 : wristCpos1;
        clawCpos1 = preset.claw1 != -1.0 ? preset.claw1 : clawCpos1;
        armCpos = preset.arm != -1.0 ? preset.arm : armCpos;
        rotationalCpos = preset.rotational != -1.0 ? preset.rotational : rotationalCpos;
        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
    }
}
