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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.V6.paths.FiveSpecAutoPaths;
import org.firstinspires.ftc.teamcode.teleOp.MainV6;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.AutoPresetStates;
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

/**
 * BeastKit V6 auto
 * Started code  @  5/26/25  @  10:59 am
 * It is a 5 specimen auto with park. It hangs a preloaded specimen then pushes the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.7, 5/26/25
**/

@Config("5 Spec Auto")
@Autonomous(name = "5+0", group = ".ftc23403")
public class FiveSpecAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    public static Integer pauses = 150;
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    ElapsedTime autoTimeE = new ElapsedTime();
    ElapsedTime transitionTimer;
    ElapsedTime timer;
    ElapsedTime subArmThrowTimer;
    private double autoTime;
    private TelemetryM telemetryM;
    public static boolean debugMode = true;
    /** store the state of our auto. **/
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
    private static CachingServo claw2; // 1x 2  5kg
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
    private static AutoPresetStates presetState = AutoPresetStates.NO_PRESET;
    SlidersStates extendArmState = SlidersStates.FLOATING;
    /* preload lines */
    boolean scoreSpecimen1Started = false;
    boolean pushBlock1Started = false;
    boolean pushBlock2Started = false;
    boolean pushBlock3Started = false;
    boolean grabSpecimen1Started = false;
    boolean scoreSpecimen2Started = false;
    boolean grabSpecimen2Started = false;
    boolean scoreSpecimen3Started = false;
    boolean grabSpecimen3Started = false;
    boolean scoreSpecimen4Started = false;
    boolean grabSpecimen4Started = false;
    boolean scoreSpecimen5Started = false;
    boolean parkStarted = false;
    boolean delay = true;
    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1: /* line1 */
                if (!scoreSpecimen1Started) {
                    submersible.subIn();
                    presetState = AutoPresetStates.SCORE_STAGE_1_FRONT;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen1(), true);
                    scoreSpecimen1Started = true;
                }
                if (!follower.isBusy()) {
                    if (Math.abs(extendArmSS.getInches1() - 16) > 2) {
                        presetState = AutoPresetStates.SCORE_STAGE_2_FRONT;
                    } else if (extendArmSS.getInches1() > 15.15) {
                        claw1.setPosition(0);
                        claw.open(1);
                    }
                    if (extendArmSS.getInches1() > 15.15) setPathState(2);
                }
                break;
            case 2: /* line2 */
                if (!pushBlock1Started) {
                    presetState = AutoPresetStates.TRANSITION;
                    follower.followPath(FiveSpecAutoPaths.pushBlock1(), true);
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
                    setPathState(292);
                }
                break;
            case 292: /* line2b */
                if (!pushBlock3Started) {
                    follower.followPath(FiveSpecAutoPaths.pushBlock3(), true);
                    pushBlock3Started = true;
                }
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5: /* line5 */
                if (!grabSpecimen1Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen1(), true);
                    grabSpecimen1Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    if (claw1.getPosition() == 1) setPathState(6);
                }
                break;
            case 6: /* line6 */
                if (!scoreSpecimen2Started) {
                    claw1.setPosition(1);
                    claw.close(1);
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen2(), true);
                    delay = false;
                    scoreSpecimen2Started = true;
                }
                if (!follower.isBusy()) {
                    presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    if (!delay) {
                        initTimer();
                        delay = true;
                    }
                    if (eDelay(pauses)) setPathState(7);
                }
                break;
            case 7: /* line7 */
                if (!grabSpecimen2Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen2(), true);
                    grabSpecimen2Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    if (claw1.getPosition() == 1) setPathState(8);
                }
                break;
            case 8: /* line8 */
                if (!scoreSpecimen3Started) {
                    claw1.setPosition(1);
                    claw.close(1);
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen3(), true);
                    delay = false;
                    scoreSpecimen3Started = true;
                }
                if (!follower.isBusy()) {
                    presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    if (!delay) {
                        initTimer();
                        delay = true;
                    }
                    if (eDelay(pauses)) setPathState(9);
                }
                break;
            case 9: /* line9 */
                if (!grabSpecimen3Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen3(), true);
                    grabSpecimen3Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    if (claw1.getPosition() == 1) setPathState(10);
                }
                break;
            case 10: /* line10 */
                if (!scoreSpecimen4Started) {
                    claw1.setPosition(1);
                    claw.close(1);
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen4(), true);
                    delay = false;
                    scoreSpecimen4Started = true;
                }
                if (!follower.isBusy()) {
                    presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    if (!delay) {
                        initTimer();
                        delay = true;
                    }
                    if (eDelay(pauses)) setPathState(11);
                }
                break;
            case 11: /* line11 */
                if (!grabSpecimen4Started) {
                    presetState = AutoPresetStates.HUMAN_PLAYER;
                    follower.followPath(FiveSpecAutoPaths.grabSpecimen4(), true);
                    grabSpecimen4Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    if (claw1.getPosition() == 1) setPathState(12);
                }
                break;
            case 12: /* line12 */
                if (!scoreSpecimen5Started) {
                    claw1.setPosition(1);
                    claw.close(1);
                    presetState = AutoPresetStates.SCORE_STAGE_1_BACK;
                    follower.followPath(FiveSpecAutoPaths.scoreSpecimen5(), true);
                    delay = false;
                    scoreSpecimen5Started = true;
                }
                if (!follower.isBusy()) {
                    presetState = AutoPresetStates.SCORE_STAGE_2_BACK;
                    if (!delay) {
                        initTimer();
                        delay = true;
                    }
                    if (eDelay(pauses)) setPathState(15);
                }
                break;
            case 15: /* line15 */
                if (!parkStarted) {
                    presetState = AutoPresetStates.TRANSITION;
                    follower.followPath(FiveSpecAutoPaths.park(), true);
                    parkStarted = true;
                }
                if (follower.isBusy() && angleDiffDegrees(Math.toDegrees(follower.getPose().getHeading()), FiveSpecAutoPaths.parkPoints.getEndHeading()) <= 2) submersible.subFull();
                if (!follower.isBusy()) setPathState(-1);
                break;
            case -1: /* done */
                autoTime = autoTimeE.seconds();
                setPathState(-2);
                break;
        }
    }
    // util
    public static double angleDiffDegrees(double a, double b) {
        double diff = ((a - b + 180) % 360 + 360) % 360 - 180;
        return Math.abs(diff);
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
                1.0,
                -1.0,
                0.0,
                0.0,
                0.5,
                0.1,
                0.0,
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
                0.1,
                0.0,
                0.0);
        public static CustomPresets scoreStage1F = new CustomPresets(
                5,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0,
                -1.0);
        public static CustomPresets scoreStage2F = new CustomPresets(
                16,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.23,
                -1.0,
                -1.0);
        public static CustomPresets scoreStage1B = new CustomPresets(
                MainV6.eaLimitLow,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.6,
                0.55,
                1.0,
                -1.0);
        public static CustomPresets scoreStage2B = new CustomPresets(
                -1,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                -1.0,
                0.75,
                -1.0,
                -1.0);
        public static CustomPresets slidesOut = new CustomPresets(
                -1,
                0.0,
                0.0,
                -1.0,
                0.0,
                -1.0,
                -1.0,
                0.0,
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
    }
    // transition
    public void transition() {
        subArmCpos = 1;
        clawCpos2 = 0.8;
        wristCpos2 = 0.65;
        armCpos = 0.27;
        clawCpos1 = 0;
        wristCpos1 = 0;
    }
    public void transition2() {
        clawCpos2 = 0.95;
        rotationalCpos1 = 1;
        wristCpos2 = 1;
    }

    public void initTimer() {
        timer.reset();
    }
    public boolean eDelay(int ms) {
        return timer.milliseconds() > ms;
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
        claw2 = new CachingServo(hardwareMap.get(Servo.class, "claw2")); // 1x 25kg
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
        // limits
        wrist1.setPosition(wristCpos1 = 1);
        wrist2.setPosition(wristCpos2 = 1);
        claw1.setPosition(clawCpos1 = 1);
        claw2.setPosition(clawCpos2 = 1);
        arm.setPosition(armCpos = 0.15);
        subArm.setPosition(subArmCpos = 1);
        rotation1.setPosition(rotationalCpos1 = 0);
        rotation2.setPosition(rotationalCpos2 = 0);
        // movement
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        transitionTimer = new ElapsedTime();
        transitionTimer.reset();
        timer = new ElapsedTime();
        timer.reset();
        subArmThrowTimer = new ElapsedTime();
        subArmThrowTimer.reset();
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
        // extendArm code
        extendArmSS.update(false, false);
        extendArmSS.setEaCorrection(eaCorrection);
        extendArmSS.setLimits(MainV6.eaLimitHigh, MainV6.eaLimitLow);
        extendArmSS.setMaxSpeedDown(MainV6.EA_MAX_SPEED_DOWN);
        extendArmState = extendArmSS.getState();
        // preset code
        switch (presetState) {
            case HUMAN_PLAYER:
                applyPreset(presets.humanPlayer);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case PARK:
                applyPreset(presets.park);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case TRANSITION:
                applyPreset(presets.transition);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case SCORE_STAGE_1_FRONT:
                applyPreset(presets.scoreStage1F);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case SCORE_STAGE_2_FRONT:
                applyPreset(presets.scoreStage2F);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case SCORE_STAGE_1_BACK:
                applyPreset(presets.scoreStage1B);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case SCORE_STAGE_2_BACK:
                applyPreset(presets.scoreStage2B);
                presetState = AutoPresetStates.NO_PRESET;
                break;
            case SUB_THROW:
                applyPreset(MainV6Presets.subThrow);
                if (subArmThrowTimer.milliseconds() > 200) {
                    presetState = AutoPresetStates.TRANSITION;
                }
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
        setPathState(1);
    }

    /** stop **/
    @Override
    public void stop() {
        LynxUtils.setLynxColor(true, true, 0, 255, 0);
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}
