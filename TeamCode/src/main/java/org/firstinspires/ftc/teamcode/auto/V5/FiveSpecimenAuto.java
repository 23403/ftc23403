package org.firstinspires.ftc.teamcode.auto.V5;

import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.teleOp.old.MainV5.eaLimitHigh;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.V5.paths.FiveSpecimenAutoPushPaths;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

/**
 * BeastKit V5 auto
 * Started code  @  4/13/25  @  3:30 pm
 * Finished code  @  4/27/25  @  4:17 pm
 * It is a 5 specimen auto with park. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.5, 5/8/25
**/

@Disabled
@Config("5 Spec Auto PUSH")
@Autonomous(name = "5+0 push", group = ".ftc23403")
public class FiveSpecimenAuto extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    public static Integer pauses = 500;
    private DashboardPoseTracker dashboardPoseTracker;
    private PoseUpdater poseUpdater;
    ElapsedTime autoTimeE = new ElapsedTime();
    private double autoTime;
    /** store the state of our auto. **/
    private int pathState;
    // servos
    private Servo swiper; // 1x goBilda speed
    private Servo arm; // 2x axon
    private Servo wrist1; // 1x axon
    private Servo claw1; // 1x goBilda speed
    private Servo submersibleArm; // 2x axon
    private Servo wrist2; // 1x axon
    private Servo claw2; // 1x goBilda speed
    private Servo rotation; // 1x goBilda speed
    // servo positions
    public static double wristCpos1 = 1;
    public static double clawCpos1 = 1;
    public static double swiperCpos = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.15;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0;
    // extendArm
    private PIDController controller;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private static double slidesTARGET = 0;
    private static ExtendArmStates extendArmState = ExtendArmStates.FLOATING;
    ElapsedTime resetTimer = new ElapsedTime();
    // Get current positions
    int eaTicks1 = 0;
    int eaTicks2 = 0;
    // Convert ticks to inches
    double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
    double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
    /* preload lines */
    boolean preloadStarted = false;
    boolean grabSpecimen1Started = false;
    boolean scoreSpecimen1Started = false;
    boolean pushBlock1Started = false;
    boolean pushBlock2Started = false;
    boolean pushBlock3Started = false;
    boolean scoreSpecimen2Started = false;
    boolean grabSpecimen2Started = false;
    boolean scoreSpecimen3Started = false;
    boolean grabSpecimen3Started = false;
    boolean scoreSpecimen4Started = false;
    boolean parkStarted = false;

    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: /* line1 */
                if (!preloadStarted) {
                    submersible.subIn();
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPushPaths.preload(), false);
                    preloadStarted = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1.setPosition(0);
                        claw.open(1);
                        setPathState(1);
                    }
                }
                break;
            case 1: /* line2 */
                if (!grabSpecimen1Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPushPaths.grabSpecimen1(), false);
                    grabSpecimen1Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    Timer.wait(pauses);
                    setPathState(2);
                }
                break;
            case 2: /* line3 */
                if (!scoreSpecimen1Started) {
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen1(), false);
                    scoreSpecimen1Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1.setPosition(0);
                        claw.open(1);
                        setPathState(3);
                    }
                }
                break;
            case 3: /* line4 */
                if (!pushBlock1Started) {
                    presets.transition();
                    follower.followPath(FiveSpecimenAutoPushPaths.pushBlock1(), true);
                    pushBlock1Started = true;
                }
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecimenAutoPushPaths.pushBlock1Points.endPointX) < 2 && Math.abs(follower.getPose().getY() - FiveSpecimenAutoPushPaths.pushBlock1Points.endPointY) < 2)) {
                    setPathState(999);
                }
                break;
            case 999: /* line4a */
                if (!pushBlock2Started) {
                    follower.followPath(FiveSpecimenAutoPushPaths.pushBlock2(), true);
                    pushBlock2Started = true;
                }
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecimenAutoPushPaths.pushBlock2Points.endPointX) < 3 && Math.abs(follower.getPose().getY() - FiveSpecimenAutoPushPaths.pushBlock2Points.endPointY) < 2)) {
                    Timer.wait(pauses);
                    setPathState(998);
                }
                break;
            case 998: /* line4b */
                if (!pushBlock3Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPushPaths.pushBlock3(), true);
                    pushBlock3Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    Timer.wait(pauses);
                    setPathState(4);
                }
                break;
            case 4: /* line5 */
                if (!scoreSpecimen2Started) {
                    presets.scoreStage1();
                    // extendArmMove(10);
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen2(), false);
                    scoreSpecimen2Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        presets.scoreStage2();
                        // extendArmMove(11);
                    } else if (Math.abs(eaInches1 - 20) <= 2) {
                        claw1.setPosition(0);
                        claw.open(1);
                        setPathState(5);
                    }
                }
                break;
            case 5: /* line6 */
                if (!grabSpecimen2Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPushPaths.grabSpecimen2(), false);
                    grabSpecimen2Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 6: /* line7 */
                if (!scoreSpecimen3Started) {
                    presets.scoreStage1();
                    // extendArmMove(10);
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen3(), false);
                    scoreSpecimen3Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        presets.scoreStage2();
                        // extendArmMove(11);
                    } else if (Math.abs(eaInches1 - 20) <= 2) {
                        claw1.setPosition(0);
                        claw.open(1);
                        setPathState(7);
                    }
                }
                break;
            case 7: /* line8 */
                if (!grabSpecimen3Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPushPaths.grabSpecimen3(), false);
                    grabSpecimen3Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw.close(1);
                    Timer.wait(pauses);
                    setPathState(8);
                }
                break;
            case 8: /* line9 */
                if (!scoreSpecimen4Started) {
                    presets.scoreStage1();
                    // extendArmMove(10);
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen4(), false);
                    scoreSpecimen4Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    if (Math.abs(eaInches1 - 18.3) > 2) {
                        presets.scoreStage2();
                        // extendArmMove(11);
                    } else if (Math.abs(eaInches1 - 20) <= 2) {
                        claw1.setPosition(0);
                        claw.open(1);
                        setPathState(9);
                    }
                }
                break;
            case 9: /* line10 */
                if (!parkStarted) {
                    presets.transition();
                    follower.followPath(FiveSpecimenAutoPushPaths.park(), false);
                    parkStarted = true;
                }
                // if (follower.isBusy() && angleDiffDegrees(Math.toDegrees(follower.getPose().getHeading()), FiveSpecimenAutoPushPaths.parkPoints.getEndHeading()) <= 2) submersible.subFull();
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
        slidesTARGET = pos;
        extendArmState = ExtendArmStates.MOVING_TO_PRESET;
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
    private static void swiper(double pos) {
        swiperCpos = pos;
    }
    private static void arm(double pos) {
        armCpos = pos;
    }
    private static void submersibleArm(double pos) {
        subArmCpos = pos;
    }
    private static void rotation(double pos) {
        rotationalCpos = pos;
    }

    /** states **/
    private static class claw {
        public static void open(int claw) {
            if (claw == 1) claw1(0);
            if (claw == 2) claw2(0);
        }
        public static void close(int claw) {
            if (claw == 1) claw1(1);
            if (claw == 2) claw2(1);
        }
    }
    private static class submersible {
        public static void subQuarterly() {
            submersibleArm(0.75);
        }
        public static void subThreeQuarters() {
            submersibleArm(0.25);
        }
        public static void subHalf() {
            submersibleArm(0.5);
        }
        public static void subFull() {
            submersibleArm(0);
        }
        public static void subIn() {
            submersibleArm(1);
        }
    }
    // @Config("5+0 PUSH PRESETS")
    private static class presets {
        public static void scoreBack() {
            extendArmMove(6.5);
            arm(0.8);
            wrist1(0.18);
            FiveSpecimenAuto.claw.close(1);
        }
        public static void scoreStage1() {
            extendArmMove(10);
            FiveSpecimenAuto.submersible.subIn();
            wrist1(0.6);
            arm(0.23);
            FiveSpecimenAuto.claw.close(1);
        }
        public static void scoreStage2() {
            extendArmMove(20);
        }
        public static void transition() {
            extendArmMove(0);
            FiveSpecimenAuto.submersible.subIn();
            wrist2(0.9);
            wrist1(0.5);
            arm(0.18);
            rotation(0.52);
        }
        public static void humanPlayer() {
            extendArmMove(0);
            wrist1(0.42);
            arm(0.96);
            FiveSpecimenAuto.claw.open(0);
        }
        public static void park() {
            extendArmMove(0);
            FiveSpecimenAuto.submersible.subFull();
            wrist2(0.9);
            wrist1(0.5);
            arm(0.18);
            rotation(0.52);
        }
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
        controller = new PIDController(Math.sqrt(P), I, D);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        LynxUtils.setLynxColor(true, true, 255, 0, 255);
        // motors
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        swiper = hardwareMap.get(Servo.class, "swiper"); // 1x goBilda speed
        // ea
        arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x goBilda speed
        // sa
        submersibleArm = hardwareMap.get(Servo.class, "subArm"); // 2x axon
        wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x axon
        claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // directions
        swiper.setDirection(Servo.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0.05, 0.88);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        submersibleArm.scaleRange(0.45, 1);
        swiper.scaleRange(0.3, 0.83);
        // extendArm
        Motors.resetEncoders(extendArm1, extendArm2);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
        resetTimer.reset();
        // starting pos
        wrist1.setPosition(1);
        wrist2.setPosition(1);
        claw1.setPosition(1);
        claw2.setPosition(1);
        arm.setPosition(0.15);
        submersibleArm.setPosition(1);
        swiper.setPosition(1);
        rotation.setPosition(0);
        // movement
        pathTimer = new com.pedropathing.util.Timer();
        opmodeTimer = new com.pedropathing.util.Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(FiveSpecimenAutoPushPaths.startPos);
        // Draw the robot on the dashboard
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(FiveSpecimenAutoPushPaths.startPos);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
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
        // servos
        if (Math.abs(wrist1.getPosition() - wristCpos1) > 0.02) wrist1.setPosition(wristCpos1);
        if (Math.abs(wrist2.getPosition() - wristCpos2) > 0.02) wrist2.setPosition(wristCpos2);
        if (Math.abs(claw1.getPosition() - clawCpos1) > 0.02) claw1.setPosition(clawCpos1);
        if (Math.abs(claw2.getPosition() - clawCpos2) > 0.02) claw2.setPosition(clawCpos2);
        if (Math.abs(arm.getPosition() - armCpos) > 0.02) arm.setPosition(armCpos);
        if (Math.abs(submersibleArm.getPosition() - subArmCpos) > 0.02) submersibleArm.setPosition(subArmCpos);
        if (Math.abs(swiper.getPosition() - swiperCpos) > 0.02) swiper.setPosition(swiperCpos);
        if (Math.abs(rotation.getPosition() - rotationalCpos) > 0.02) rotation.setPosition(rotationalCpos);
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
        if (Math.abs(eaInches1 - 0) > 2 && (extendArmState == ExtendArmStates.PRESET_REACHED || extendArmState == ExtendArmStates.ZERO_POS_RESET ||  extendArmState == ExtendArmStates.MAX_POS)) {
            extendArm1.setPower(ff);
            extendArm2.setPower(ff);
        }
        // states
        if (Math.abs(eaInches1 - eaLimitHigh) < 1 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
            extendArmState = ExtendArmStates.MAX_POS;
        } else if (Math.abs(eaInches1 - 0) < 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET && extendArmState != ExtendArmStates.RESETTING_ZERO_POS && extendArmState != ExtendArmStates.ZERO_POS_RESET && extendArmState != ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            extendArmState = ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION;
            resetTimer.reset();
        }
        // pre resetting slides pos
        if (extendArmState == ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
            if (resetTimer.milliseconds() > 200 && Math.abs(eaInches1 - 0) < 2) {
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
        // telemetry for debugging
        if (pathState == -2) telemetry.addData("Time took:", autoTime);
        telemetry.addData("currentState", extendArmState);
        telemetry.addData("extendArm1 Power", extendArm1.getPower());
        telemetry.addData("extendArm2 Power", extendArm2.getPower());
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