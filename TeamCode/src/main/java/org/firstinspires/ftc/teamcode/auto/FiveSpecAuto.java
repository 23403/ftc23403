package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;
import static org.firstinspires.ftc.teamcode.teleOp.MainV5.eaLimitHigh;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.paths.FiveSpecimenAutoPaths;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;
import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

/**
 * BeastKit V5 specimen auto
 * Started code  @  4/24/25  @  11:56 am
 * Expected to finish code  @  4/26/25
 * It is a 5 specimen auto with park. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.4, 4/24/25
**/

@Config("5 Spec Auto")
@Autonomous(name = "5+0", group = ".ftc23403")
public class FiveSpecAuto extends OpMode {
    private Follower follower;
    private com.pedropathing.util.Timer pathTimer, opmodeTimer;
    public static double speed = 1;
    public static Integer pauses = 200;
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
    private Servo claw1; // 1x axon
    private Servo submersibleArm1; // 1x axon
    private Servo submersibleArm2; // 1x axon
    private Servo wrist2; // 1x 20kg
    private Servo claw2; // 1x goBilda speed
    private Servo rotation; // 1x goBilda speed
    // servo positions
    public static double wristCpos1 = 0;
    public static double clawCpos1 = 1;
    public static double swiperCpos = 1;
    public static double wristCpos2 = 0.9;
    public static double clawCpos2 = 0.5;
    public static double armCpos = 0.23;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0.5;
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

    /** movements **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 1: /* line1 */
                if (!scoreSpecimen1Started) {
                    autoTimeE.reset();
                    presets.scoreBack();
                    follower.followPath(FiveSpecimenAutoPaths.scoreSpecimen1(), false);
                    scoreSpecimen1Started = true;
                }
                if (!follower.isBusy()) {
                    claw1(0);
                    setPathState(291);
                }
                break;
            case 291: /* line2a */
                if (!pushBlock1Started) {
                    submersible.subQuarterly();
                    follower.followPath(FiveSpecimenAutoPaths.pushBlock1(), false);
                    pushBlock1Started = true;
                }
                if (follower.isBusy() && follower.getPose().getX() >= 38 && follower.getPose().getY() <= 47) {
                    presets.transition();
                    submersible.subFull();
                }
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecimenAutoPaths.pushBlock1Points.endPointX) < 2 && Math.abs(follower.getPose().getY() - FiveSpecimenAutoPaths.pushBlock1Points.endPointY) < 2)) {
                    setPathState(292);
                }
                break;
            case 292: /* line2b */
                if (!pushBlock2Started) {
                    submersible.subQuarterly();
                    follower.followPath(FiveSpecimenAutoPaths.pushBlock2(), false);
                    pushBlock2Started = true;
                }
                if (follower.isBusy() && follower.getPose().getX() >= 39 && follower.getPose().getY() <= 36) submersible.subFull();
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecimenAutoPaths.pushBlock2Points.endPointX) < 2 && Math.abs(follower.getPose().getY() - FiveSpecimenAutoPaths.pushBlock2Points.endPointY) < 2)) {
                    setPathState(293);
                }
                break;
            case 293: /* line2c */
                if (!pushBlock3Started) {
                    submersible.subQuarterly();
                    follower.followPath(FiveSpecimenAutoPaths.pushBlock3(), false);
                    pushBlock3Started = true;
                }
                if (follower.isBusy() && follower.getPose().getX() >= 44 && follower.getPose().getY() <= 29) submersible.subFull();
                if (!follower.isBusy() || (Math.abs(follower.getPose().getX() - FiveSpecimenAutoPaths.pushBlock3Points.endPointX) < 2 && Math.abs(follower.getPose().getY() - FiveSpecimenAutoPaths.pushBlock3Points.endPointY) < 2)) {
                    setPathState(3);
                }
                break;
            case 3: /* line3 */
                if (!grabSpecimen1Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPaths.grabSpecimen1(), false);
                    grabSpecimen1Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(4);
                }
                break;
            case 4: /* line4 */
                if (!scoreSpecimen2Started) {
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPaths.scoreSpecimen2(), false);
                    scoreSpecimen2Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    extendArmMove(19);
                    if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(5);
                    }
                }
                break;
            case 5: /* line5 */
                if (!grabSpecimen2Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPaths.grabSpecimen2(), false);
                    grabSpecimen2Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 6: /* line6 */
                if (!scoreSpecimen3Started) {
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPaths.scoreSpecimen3(), false);
                    scoreSpecimen3Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    extendArmMove(19);
                    if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(7);
                    }
                }
                break;
            case 7: /* line7 */
                if (!grabSpecimen3Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPaths.grabSpecimen3(), false);
                    grabSpecimen3Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(8);
                }
                break;
            case 8: /* line8 */
                if (!scoreSpecimen4Started) {
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPaths.scoreSpecimen4(), false);
                    scoreSpecimen4Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    extendArmMove(19);
                    if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(9);
                    }
                }
                break;
            case 9: /* line9 */
                if (!grabSpecimen4Started) {
                    presets.humanPlayer();
                    follower.followPath(FiveSpecimenAutoPaths.grabSpecimen4(), false);
                    grabSpecimen4Started = true;
                }
                if (!follower.isBusy()) {
                    claw1.setPosition(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(10);
                }
                break;
            case 10: /* line10 */
                if (!scoreSpecimen5Started) {
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPaths.scoreSpecimen5(), false);
                    scoreSpecimen5Started = true;
                }
                if (!follower.isBusy() && extendArmState == ExtendArmStates.PRESET_REACHED) {
                    extendArmMove(19);
                    if (Math.abs(eaInches1 - 19) <= 2) {
                        claw1(0);
                        setPathState(11);
                    }
                }
                break;
            case 11: /* line11 */
                if (!parkStarted) {
                    presets.transition();
                    follower.followPath(FiveSpecimenAutoPaths.park(), false);
                    parkStarted = true;
                }
                if (follower.isBusy() && Math.abs(Math.toDegrees(follower.getPose().getHeading()) - FiveSpecimenAutoPaths.parkPoints.getEndHeading()) <= 2) submersible.subFull();
                if (!follower.isBusy()) setPathState(-1);
                break;
            case -1: /* done */
                autoTime = autoTimeE.seconds();
                break;
        }
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
    private static void sweeper(double pos) {
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
    private static class submersible {
        public static void subQuarterly() {
            submersibleArm(0.75);
        }
        public static void subFull() {
            submersibleArm(0);
        }
        public static void subIn() {
            submersibleArm(0);
        }
    }
    private static class presets {
        public static void scoreBack() {
            extendArmMove(6.5);
            arm(0.8);
            wrist1(0.18);
            claw1(1);
        }
        public static void scoreStage1() {
            extendArmMove(10);
            submersibleArm(1);
            wrist1(0.6);
            arm(0.23);
            claw1(1);
        }
        public static void scoreStage2() {
            extendArmMove(19);
        }
        public static void transition() {
            extendArmMove(0);
            submersibleArm(1);
            wrist2(0.9);
            wrist1(0.5);
            arm(0.18);
            rotation(0.52);
        }
        public static void humanPlayer() {
            extendArmMove(0);
            wrist1(0.42);
            arm(0.96);
            claw1(0);
        }
        public static void park() {
            extendArmMove(0);
            submersibleArm(0);
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
        autoTimeE.startTime();
        // motors
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        swiper = hardwareMap.get(Servo.class, "swiper"); // 1x goBilda torque
        // ea
        arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x goBilda speed
        // sa
        submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // 1x axon
        submersibleArm2 = hardwareMap.get(Servo.class, "subArm2"); // 1x axon
        wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x 25kg
        claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // directions
        swiper.setDirection(Servo.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0, 0.8);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        submersibleArm1.scaleRange(0.42, 1);
        // extendArm
        Motors.resetEncoders(List.of(extendArm1, extendArm2));
        Motors.setMode(List.of(extendArm1, extendArm2), DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        resetTimer.reset();
        // starting pos
        claw1.setPosition(1);
        claw1(1);
        // movement
        pathTimer = new com.pedropathing.util.Timer();
        opmodeTimer = new com.pedropathing.util.Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(FiveSpecimenAutoPaths.startPos);
        // Draw the robot on the dashboard
        poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(FiveSpecimenAutoPaths.startPos);
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
        wrist1.setPosition(wristCpos1);
        wrist2.setPosition(wristCpos2);
        claw1.setPosition(clawCpos1);
        claw2.setPosition(clawCpos2);
        arm.setPosition(armCpos);
        submersibleArm1.setPosition(subArmCpos);
        swiper.setPosition(swiperCpos);
        rotation.setPosition(rotationalCpos);
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
        // telemetry for debugging
        if (pathState == -1) telemetry.addData("Time took:", autoTime);
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
        setPathState(1);
    }

    /** stop **/
    @Override
    public void stop() {
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}
