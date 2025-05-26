package org.firstinspires.ftc.teamcode.auto.V6;

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
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.V6.paths.FiveSpecAutoPaths;
import org.firstinspires.ftc.teamcode.teleOp.MainV6;
import org.firstinspires.ftc.teamcode.utils.CombinedServo;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.AutoPresetStates;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.subsystems.SlidesSS;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Timer;

/**
 * BeastKit V6 auto
 * Started code  @  5/26/25  @  10:59 am
 * It is a 5 specimen auto with park. It hangs a preloaded specimen then pushes the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.6, 5/26/25
 **/

@Config("5 Spec Auto PUSH")
@Autonomous(name = "5+0 push", group = ".ftc23403")
public class FiveSpecAuto extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    public static Integer pauses = 500;
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    ElapsedTime autoTimeE = new ElapsedTime();
    private double autoTime;
    private TelemetryM telemetryM;
    public static boolean debugMode = true;
    /** store the state of our auto. **/
    private int pathState;
    // servos
    CachingServo arm1; // 1x axon max
    CachingServo arm2; // 1x axon max
    private static CachingServo wrist1; // 1x axon mini
    private static CachingServo claw1; // 1x axon mini
    private static CachingServo rotation1; // 1x axon max
    private static CombinedServo arm; // 2x axon max
    CachingServo submersibleArm1; // 1x axon max
    CachingServo submersibleArm2; // 1x 25kg
    private static CachingServo wrist2; // 1x axon mini
    private static CachingServo claw2; // 1x axon mini
    private static CachingServo rotation2; // 1x axon max
    private static CombinedServo subArm; // 1x axon max : 1x 25kg
    // servo positions
    public static double wristCpos1 = 1;
    public static double clawCpos1 = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.15;
    public static double subArmCpos = 1;
    public static double rotationalCpos2 = 0;
    public static double rotationalCpos1 = 0;
    private AnalogInput subArms;
    private AnalogInput arms;
    // analog
    private double subArmPos;
    private double armPos;
    // extend arm
    public static double slidesTARGET = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    private CachingDcMotorEx extendArm1;
    private CachingDcMotorEx extendArm2;
    private static AutoPresetStates presetState = AutoPresetStates.NO_PRESET;
    private static SlidersStates extendArmState = SlidersStates.FLOATING;
    /* preload lines */
    boolean scoreSpecimen1Started = false;
    boolean pushBlock1Started = false;
    boolean pushBlock2Started = false;
    boolean pushBlock3Started = false;
    boolean scoreSpecimen2Started = false;
    boolean grabSpecimen3Started = false;
    boolean scoreSpecimen3Started = false;
    boolean grabSpecimen4Started = false;
    boolean scoreSpecimen4Started = false;
    boolean grabSpecimen5Started = false;
    boolean scoreSpecimen5Started = false;
    boolean parkStarted = false;

    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1: /* line1 */
                if (!scoreSpecimen1Started) {
                    FiveSpecAuto.submersible.subIn();
                    presetState = AutoPresetStates.SCORE_STAGE_1_FRONT;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen1(), true);
                    scoreSpecimen1Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (Math.abs(extendArmSS.getInches1() - 18.3) > 2) {
                        presetState = AutoPresetStates.SCORE_STAGE_2_FRONT;
                    } else if (Math.abs(extendArmSS.getInches1() - 19) <= 2) {
                        FiveSpecAuto.claw.open(1);
                        setPathState(2);
                    }
                }
                break;
            case 2: /* line2 */
                if (!pushBlock1Started) {
                    presetState = AutoPresetStates.TRANSITION;
                    follower.followPath(FiveSpecAutoPaths.returnAndPushBlock1(), true);
                    pushBlock1Started = true;
                }
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecAutoPaths.pushBlock1Points.endPointX) < 2 && Math.abs(follower.getPose().getY() - FiveSpecAutoPaths.pushBlock1Points.endPointY) < 2)) {
                    setPathState(291);
                }
                break;
            case 291: /* line2a */
                if (!pushBlock2Started) {
                    follower.followPath(FiveSpecAutoPaths.pushBlock2(), true);
                    pushBlock2Started = true;
                }
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecAutoPaths.pushBlock2Points.endPointX) < 3 && Math.abs(follower.getPose().getY() - FiveSpecAutoPaths.pushBlock2Points.endPointY) < 2)) {
                    Timer.wait(pauses);
                    setPathState(292);
                }
                break;
            case 292: /* line2b */
                if (!pushBlock3Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.pushBlock3(), true);
                    pushBlock3Started = true;
                }
                if (!follower.isBusy()) {
                    FiveSpecAuto.claw.close(1);
                    Timer.wait(pauses);
                    setPathState(3);
                }
                break;
            case 3: /* line3 */
                if (!scoreSpecimen2Started) {
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen2(), true);
                    scoreSpecimen2Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (armPos > 10) {
                        FiveSpecAuto.claw.open(1);
                        setPathState(4);
                    } else {
                        presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    }
                }
                break;
            case 4: /* line4 */
                if (!grabSpecimen3Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen3(), true);
                    grabSpecimen3Started = true;
                }
                if (!follower.isBusy()) {
                    FiveSpecAuto.claw.close(1);
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 5: /* line5 */
                if (!scoreSpecimen3Started) {
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen3(), true);
                    scoreSpecimen3Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (armPos > 10) {
                        FiveSpecAuto.claw.open(1);
                        setPathState(6);
                    } else {
                        presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    }
                }
                break;
            case 6: /* line8 */
                if (!grabSpecimen4Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen4(), true);
                    grabSpecimen4Started = true;
                }
                if (!follower.isBusy()) {
                    FiveSpecAuto.claw.close(1);
                    Timer.wait(pauses);
                    setPathState(7);
                }
                break;
            case 7: /* line7 */
                if (!scoreSpecimen4Started) {
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen4(), true);
                    scoreSpecimen4Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (armPos > 10) {
                        FiveSpecAuto.claw.open(1);
                        setPathState(8);
                    } else {
                        presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    }
                }
                break;
            case 8: /* line8 */
                if (!grabSpecimen5Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen5(), true);
                    grabSpecimen5Started = true;
                }
                if (!follower.isBusy()) {
                    FiveSpecAuto.claw.close(1);
                    Timer.wait(pauses);
                    setPathState(9);
                }
                break;
            case 9: /* line9 */
                if (!scoreSpecimen5Started) {
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen5(), true);
                    scoreSpecimen5Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (armPos > 10) {
                        FiveSpecAuto.claw.open(1);
                        setPathState(10);
                    } else {
                        presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    }
                }
                break;
            case 10: /* line10 */
                if (!parkStarted) {
                    presetState = AutoPresetStates.TRANSITION;
                    follower.followPath(FiveSpecAutoPaths.park(), true);
                    parkStarted = true;
                }
                // if (follower.isBusy() && angleDiffDegrees(Math.toDegrees(follower.getPose().getHeading()), FiveSpecAutoPaths.parkPoints.getEndHeading()) <= 2) submersible.subFull();
                if (!follower.isBusy()) setPathState(-1);
                break;
            case -1: /* done */
                autoTime = autoTimeE.seconds();
                setPathState(-2);
                break;
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

    /** states **/
    private static class claw {
        public static void open(int claw) {
            if (claw == 1) {
                claw1.setPosition(0);
                claw1(0);
            }
            if (claw == 2) {
                claw2.setPosition(0);
                claw2(0);
            }
        }
        public static void close(int claw) {
            if (claw == 1) {
                claw1.setPosition(1);
                claw1(1);
            }
            if (claw == 2) {
                claw2.setPosition(0);
                claw2(1);
            }
        }
    }
    private static class submersible {
        public static void subQuarterly() {
            subArm.setPosition(0.75);
            submersibleArm(0.75);
        }
        public static void subThreeQuarters() {
            subArm.setPosition(0.25);
            submersibleArm(0.25);
        }
        public static void subHalf() {
            subArm.setPosition(0.5);
            submersibleArm(0.5);
        }
        public static void subFull() {
            subArm.setPosition(0);
            submersibleArm(0);
        }
        public static void subIn() {
            subArm.setPosition(1);
            submersibleArm(1);
        }
    }
    @Config("5+0 PRESETS")
    private static class presets {
        public static CustomPresets humanPlayer = new CustomPresets(
                MainV6.eaLimitLow,
                -1.0,
                -1.0,
                0.0,
                -1.0,
                0.42,
                0.96,
                -1.0,
                0.0);
        public static CustomPresets park = new CustomPresets(
                MainV6.eaLimitLow,
                0.0,
                1.0,
                0.0,
                0.9,
                0.5,
                0.18,
                0.0,
                0.52);
        public static CustomPresets transition = new CustomPresets(
                MainV6.eaLimitLow,
                1.0,
                1.0,
                0.0,
                1.0,
                0.5,
                0.18,
                0.52,
                0.0);
        public static CustomPresets scoreStage1F = new CustomPresets(
                10,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0,
                0.0);
        public static CustomPresets scoreStage2F = new CustomPresets(
                20,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0);
        public static CustomPresets scoreStage1B = new CustomPresets(
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0);
        public static CustomPresets scoreStage2B = new CustomPresets(
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0);
    }

    // preset controls
    public void applyPreset(CustomPresets preset) {
        // use correction code cuz its easier fr fr
        extendArmMove(preset.extendArm != -1.0 ? preset.extendArm : extendArmSS.getInches1());
        submersibleArm(preset.subArm != -1.0 ? preset.subArm : subArmCpos);
        claw2(preset.claw2 != -1.0 ? preset.claw2 : clawCpos2);
        wrist2(preset.wrist2 != -1.0 ? preset.wrist2 : wristCpos2);
        wrist1(preset.wrist1 != -1.0 ? preset.wrist1 : wristCpos1);
        claw1(preset.claw1 != -1.0 ? preset.claw1 : clawCpos1);
        arm(preset.arm != -1.0 ? preset.arm : armCpos);
        rotation2(preset.rotational2 != -1.0 ? preset.rotational2 : rotationalCpos2);
        rotation1(preset.rotational1 != -1.0 ? preset.rotational1 : rotationalCpos1);
        presetState = AutoPresetStates.NO_PRESET;
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
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = new TelemetryM(telemetry, debugMode);
        PID controller = new PID(Math.sqrt(P), I, D);
        // motors
        extendArm1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm1"));
        extendArm2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm2"));
        // servos
        subArms = hardwareMap.get(AnalogInput.class, "subArm1");
        arms = hardwareMap.get(AnalogInput.class, "arm1");
        // ea
        arm1 = new CachingServo(hardwareMap.get(Servo.class, "arm1")); // 1x axon max
        arm2 = new CachingServo(hardwareMap.get(Servo.class, "arm2")); // 1x axon max
        wrist1 = new CachingServo(hardwareMap.get(Servo.class, "wrist1")); // 1x axon mini
        claw1 = new CachingServo(hardwareMap.get(Servo.class, "claw1")); // 1x axon mini
        rotation1 = new CachingServo(hardwareMap.get(Servo.class, "rotation2")); // 1x axon max
        arm = new CombinedServo(arm1, arm2); // 2x axon max
        // sa
        submersibleArm1 = new CachingServo(hardwareMap.get(Servo.class, "subArm1")); // 1x axon max
        submersibleArm2 = new CachingServo(hardwareMap.get(Servo.class, "subArm2")); // 1x 25kg
        wrist2 = new CachingServo(hardwareMap.get(Servo.class, "wrist2")); // 1x axon mini
        claw2 = new CachingServo(hardwareMap.get(Servo.class, "claw2")); // 1x axon mini
        rotation2 = new CachingServo(hardwareMap.get(Servo.class, "rotation1")); // 1x axon max
        subArm = new CombinedServo(submersibleArm1, submersibleArm2); // 1x axon max : 1x 25kg
        // analog
        subArmPos = subArms.getVoltage() / 3.3 * 360;
        armPos = arms.getVoltage() / 3.3 * 360;
        // directions
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0.05, 0.8);
        rotation2.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        subArm.scaleRange(0.385, 0.85);
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
        follower.setStartingPose(FiveSpecAutoPaths.startPos);
        // Draw the robot on the dashboard
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(FiveSpecAutoPaths.startPos);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
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
        // misc
        telemetryM.setDebug(debugMode);
        // servos
        wrist1.setPosition(wristCpos1);
        wrist2.setPosition(wristCpos2);
        claw1.setPosition(clawCpos1);
        claw2.setPosition(clawCpos2);
        arm.setPosition(armCpos);
        subArm.setPosition(subArmCpos);
        rotation1.setPosition(rotationalCpos1);
        rotation2.setPosition(rotationalCpos2);
        // analog
        subArmPos = subArms.getVoltage() / 3.3 * 360;
        armPos = arms.getVoltage() / 3.3 * 360;
        // extendArm code
        extendArmSS.update(false, false);
        extendArmSS.setEaCorrection(eaCorrection);
        extendArmSS.setLimits(MainV6.eaLimitHigh, MainV6.eaLimitLow);
        extendArmState = extendArmSS.getState();
        // preset code
        switch (presetState) {
            case HUMAN_PLAYER:
                applyPreset(FiveSpecAuto.presets.humanPlayer);
                break;
            case PARK:
                applyPreset(FiveSpecAuto.presets.park);
                break;
            case TRANSITION:
                applyPreset(FiveSpecAuto.presets.transition);
                break;
            case SCORE_STAGE_1_FRONT:
                applyPreset(presets.scoreStage1F);
                break;
            case SCORE_STAGE_2_FRONT:
                applyPreset(presets.scoreStage2F);
                break;
            case SCORE_STAGE_1_BACK:
                applyPreset(presets.scoreStage1B);
                break;
            case SCORE_STAGE_2_BACK:
                applyPreset(presets.scoreStage2B);
                break;
        }
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        if (pathState == -2) telemetryM.addData("Time took:", autoTime);
        telemetryM.addData(true, "currentState", extendArmSS.getState());
        telemetryM.addData(true, "extendArm1 Power", extendArm1.getPower());
        telemetryM.addData(true, "extendArm2 Power", extendArm2.getPower());
        telemetryM.addData(true, "PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
        telemetryM.addData(true, "target", slidesTARGET);
        telemetryM.addData(true, "eaCpos1", extendArmSS.getInches1());
        telemetryM.addData(true, "eaCpos2", extendArmSS.getInches2());
        telemetryM.addData(true, "error1", Math.abs(slidesTARGET - extendArmSS.getInches1()));
        telemetryM.addData(true, "error2", Math.abs(slidesTARGET - extendArmSS.getInches2()));
        telemetryM.addData(true, "errorAvg", (Math.abs(slidesTARGET - extendArmSS.getInches1()) + Math.abs(slidesTARGET - extendArmSS.getInches2())) / 2);
        telemetryM.addData(true, "Submersible Arm Position:", subArm.getPosition());
        telemetryM.addData(true, "Wrist Position1:", wrist1.getPosition());
        telemetryM.addData(true, "Wrist Position2:", wrist2.getPosition());
        telemetryM.addData(true, "Claw Position1:", claw1.getPosition());
        telemetryM.addData(true, "Claw Position2:", claw2.getPosition());
        telemetryM.addData(true, "Arm Position:", arm.getPosition());
        telemetryM.addData(true, "Rotation Position1:", rotation1.getPosition());
        telemetryM.addData(true, "Rotation Position2:", rotation2.getPosition());
        telemetryM.addData(true, "Control Hub Current", LynxUtils.getControlHubCurrent());
        telemetryM.addData(true, "Expansion Hub Current", LynxUtils.getExpansionHubCurrent());
        telemetryM.addData(true, "Servo Hub Current", LynxUtils.getServoHubCurrent());
        telemetryM.addData(true, "path state", pathState);
        telemetryM.addData(true, "x", follower.getPose().getX());
        telemetryM.addData(true, "y", follower.getPose().getY());
        telemetryM.addData(true, "heading", follower.getPose().getHeading());
        telemetryM.update();
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
        LynxUtils.setLynxColor(true, true, 0, 255, 0);
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}
