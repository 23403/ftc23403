/***
 * MAIN V6
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch because i hate my old code again
 * based off of MainV5 but better
 * started recoding at 5/12/25  @  11:38 am
 * robot v6 expected to be finished building by 5/23/25
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.LimelightState;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.enums.ModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.PresetStates;
import org.firstinspires.ftc.teamcode.variables.enums.RotationStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.BasketsModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.SpecModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.SubModeStates;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.subsystems.SlidesSS;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Sensor;
import xyz.nin1275.utils.Timer;

@Configurable
@Config("MainV6")
@TeleOp(name="Main v6", group=".ftc23403")
public class MainV6 extends LinearOpMode {
    /**
     * @TODO add in everything
     * @TODO fix all the movements using state systems for everything
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
    public static double rotationalCpos = 0;
    // misc
    public static boolean redSide = true;
    public static boolean debugMode = false;
    public static double wheelSpeedMax = 1;
    public static double wheelSpeedMinEA = 0.7;
    public static double wheelSpeedMinSA = 0.8;
    private double wheelSpeed = wheelSpeedMax;
    public static double hangPower = 0.8;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static double slidesTARGET = 0;
    public static double eaLimitHigh = 33.6;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    boolean openClaw = false;
    // states
    SlidersStates extendArmState = SlidersStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
    private static RotationStates rotationState = RotationStates.MIDDLE;
    LimelightState llState = LimelightState.INIT;
    private static ModeStates mode = ModeStates.SUBMERSIBLE;
    private static SubModeStates subStates = SubModeStates.RETURN;
    private static SpecModeStates specStates = SpecModeStates.GRAB;
    private static BasketsModeStates basketsStates = BasketsModeStates.RETURN;
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
        submersibleArm.setPosition(1);
        rotation.setPosition(0);
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
                if (Math.abs(submersibleArm.getPosition() - subArmCpos) > 0.02) submersibleArm.setPosition(subArmCpos);
                if (Math.abs(rotation.getPosition() - rotationalCpos) > 0.02) rotation.setPosition(rotationalCpos);
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
                // claw
                clawCpos1 = openClaw ? 0 : 1;
                // extendArm code
                extendArmSS.update(gamepad2.right_stick_y > 0.1, gamepad2.right_stick_y < 0.1);
                // submersibleArm code
                if (gamepad1.right_stick_y > 0.1) {
                    subArmCpos = 0;
                } else if (gamepad1.right_stick_y < 0.1) {
                    subArmCpos = 1;
                }
                // preset code
                switch (subStates) {
                    case MOVE_OUT:
                        subArmCpos = 0;
                        if (true) wristCpos2 = 0.8;
                        break;
                    case GRAB:
                        wristCpos2 = 1;
                        Timer.wait(100);
                        clawCpos2 = 1;
                        break;
                    case RETURN:
                        applyPreset(MainV5.presets.transition);
                        presetState = PresetStates.TRANSITION;
                        break;
                }
                /**
                 * GAMEPAD 1
                 *   X / ▢         - N/A
                 *   Y / Δ         - N/A
                 *   B / O         - N/A
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

                /**
                 * GAMEPAD 2
                 *   X / ▢         - N/A
                 *   Y / Δ         - N/A
                 *   B / O         - N/A
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
                // modes
                if (gamepad1.dpad_down) mode = ModeStates.SUBMERSIBLE;
                if (gamepad2.dpad_up) mode = ModeStates.BASKETS;
                if (gamepad2.dpad_down) {
                    mode = ModeStates.SPECIMEN;
                    presetState = PresetStates.HUMAN_PLAYER;
                }
                // controls
                switch (mode) {
                    case SUBMERSIBLE:
                        if (currentGamepad1.x && !previousGamepad1.x) {
                            switch (subStates) {
                                case MOVE_OUT:
                                    subStates = SubModeStates.GRAB;
                                    break;
                                case GRAB:
                                    subStates = SubModeStates.RETURN;
                                    break;
                                case RETURN:
                                    subStates = SubModeStates.MOVE_OUT;
                                    break;
                            }
                        }
                        if (currentGamepad1.b && !previousGamepad1.b) {
                            switch (subStates) {
                                case MOVE_OUT:
                                case THROW:
                                    subStates = SubModeStates.RETURN;
                                    break;
                                case GRAB:
                                    subStates = SubModeStates.MOVE_OUT;
                                    break;
                                case RETURN:
                                    subStates = SubModeStates.THROW;
                                    break;
                            }
                        }
                        if (currentGamepad1.y && !previousGamepad1.y) {
                            subStates = SubModeStates.RETURN;
                        }
                        break;
                    case SPECIMEN:
                        if (currentGamepad2.b && !previousGamepad2.b) {
                            switch (specStates) {
                                case PRE_SPECIMEN:
                                    specStates = SpecModeStates.SCORE;
                                    break;
                                case GRAB:
                                case RETURN:
                                    specStates = SpecModeStates.PRE_SPECIMEN;
                                    break;
                            }
                        }
                        if (currentGamepad2.x && !previousGamepad2.x) {
                            switch (specStates) {
                                case PRE_SPECIMEN:
                                case RETURN:
                                    specStates = SpecModeStates.GRAB;
                                    break;
                            }
                        }
                        if (currentGamepad2.y && !previousGamepad2.y) {
                            specStates = SpecModeStates.RETURN;
                        }
                        break;
                    case BASKETS:
                        break;
                }
                // rotate
                if (rotationalCpos > 0.51) rotationState = RotationStates.RIGHT;
                if (rotationalCpos < 0.49) rotationState = RotationStates.LEFT;
                if (rotationalCpos > 0.49 && rotationalCpos < 0.51) rotationState = RotationStates.MIDDLE;
                if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)) {
                    rotationalCpos = 0.5;
                    rotationState = RotationStates.MIDDLE;
                } else if (currentGamepad1.left_trigger > 0 && !(previousGamepad1.left_trigger > 0)) {
                    rotationalCpos = 0;
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
                telemetryM.addData(true, "DEBUG:", "PickUp " + (Sensor.pickUpRed() ? "RED" : Sensor.pickUpBlue() ? "BLUE" : Sensor.pickUpYellow() ? "YELLOW" : "NONE"));
                telemetryM.addData(true, "DEBUG:", "Grabbed " + (Sensor.isRedGrabbed() ? "RED" : Sensor.isBlueGrabbed() ? "BLUE" : Sensor.isYellowGrabbed() ? "YELLOW" : "NONE"));
                telemetryM.addData(true, "Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
                telemetryM.addData(true, "Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
                telemetryM.addData(true, "Submersible Arm Position:", submersibleArm.getPosition());
                telemetryM.addData(true, "Wrist Position1:", wrist1.getPosition());
                telemetryM.addData(true, "Wrist Position2:", wrist2.getPosition());
                telemetryM.addData(true, "Claw Position1:", claw1.getPosition());
                telemetryM.addData(true, "Claw Position2:", claw2.getPosition());
                telemetryM.addData(true, "Arm Position:", arm.getPosition());
                telemetryM.addData(true, "Rotation Position:", rotation.getPosition());
                telemetryM.addData(true, "Red side?", redSide);
                telemetryM.addData(true, "slides reset timer", extendArmSS.getResetTimer().milliseconds());
                telemetryM.addData(true, "extendArm1 Power", extendArm1.getPower());
                telemetryM.addData(true, "extendArm2 Power", extendArm2.getPower());
                telemetryM.update();
            }
        }
        if (isStopRequested()) {
            // stop code
            LynxUtils.setLynxColor(true, true, 0, 255, 0);
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
