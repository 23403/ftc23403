package org.firstinspires.ftc.teamcode.auto;

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

import org.firstinspires.ftc.teamcode.auto.paths.FiveSpecimenAutoPushPaths;
import org.firstinspires.ftc.teamcode.teleOp.MainV5;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.subsystems.SlidesSS;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Timer;

/**
 * BeastKit V5 auto
 * Started code  @  4/13/25  @  3:30 pm
 * Finished code  @  4/27/25  @  4:17 pm
 * It is a 5 specimen auto with park. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.4, 4/27/25
**/

@Config("5 Spec Auto PUSH")
@Autonomous(name = "5+0 push", group = ".ftc23403")
public class FiveSpecimenAuto extends OpMode {
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
    private Servo wrist2; // 1x axon
    private Servo claw2; // 1x goBilda speed
    private Servo rotation; // 1x goBilda speed
    // servo positions
    public static double wristCpos1 = 0;
    public static double clawCpos1 = 1;
    public static double swiperCpos = 0;
    public static double wristCpos2 = 0.9;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.23;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0.5;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private static SlidersStates extendArmState = SlidersStates.FLOATING;
    private static SlidesSS extendArmSS;
    public static boolean eaCorrection = true;
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
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (Math.abs(extendArmSS.getInches1() - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(extendArmSS.getInches1() - 19) <= 2) {
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
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (Math.abs(extendArmSS.getInches1() - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(extendArmSS.getInches1() - 19) <= 2) {
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
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen2(), false);
                    scoreSpecimen2Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (Math.abs(extendArmSS.getInches1() - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(extendArmSS.getInches1() - 19) <= 2) {
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
                    claw.close(1);
                    claw1(1);
                    Timer.wait(pauses);
                    setPathState(6);
                }
                break;
            case 6: /* line7 */
                if (!scoreSpecimen3Started) {
                    presets.scoreStage1();
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen3(), false);
                    scoreSpecimen3Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (Math.abs(extendArmSS.getInches1() - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(extendArmSS.getInches1() - 19) <= 2) {
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
                    follower.followPath(FiveSpecimenAutoPushPaths.scoreSpecimen4(), false);
                    scoreSpecimen4Started = true;
                }
                if (!follower.isBusy() && extendArmState == SlidersStates.PRESET_REACHED) {
                    if (Math.abs(extendArmSS.getInches1() - 18.3) > 2) {
                        presets.scoreStage2();
                    } else if (Math.abs(extendArmSS.getInches1() - 19) <= 2) {
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
                if (follower.isBusy() && angleDiffDegrees(Math.toDegrees(follower.getPose().getHeading()), FiveSpecimenAutoPushPaths.parkPoints.getEndHeading()) <= 2) submersible.subFull();
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
            extendArmMove(19);
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
        // extendArm
        PID controller = new PID(Math.sqrt(P), I, D);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
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
        wrist2.scaleRange(0.05, 0.88);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        submersibleArm1.scaleRange(0.45, 1);
        swiper.scaleRange(0.3, 0.83);
        // extendArm
        extendArmSS = new SlidesSS(extendArm1, extendArm2, controller, K, F, CPR, INCHES_PER_REV, MainV5.eaLimitHigh, MainV5.eaLimitLow, eaCorrection, false);
        // starting pos
        claw1.setPosition(1);
        arm.setPosition(0.23);
        wrist1.setPosition(0.5);
        claw1(1);
        arm(0.23);
        wrist1(0.5);
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
        wrist1.setPosition(wristCpos1);
        wrist2.setPosition(wristCpos2);
        claw1.setPosition(clawCpos1);
        claw2.setPosition(clawCpos2);
        arm.setPosition(armCpos);
        submersibleArm1.setPosition(subArmCpos);
        swiper.setPosition(swiperCpos);
        rotation.setPosition(rotationalCpos);
        // extendArm code
        extendArmState = extendArmSS.getState();
        extendArmSS.setEaCorrection(eaCorrection);
        extendArmSS.setLimits(MainV5.eaLimitHigh, MainV5.eaLimitLow);
        extendArmSS.update();
        // telemetry for debugging
        if (pathState == -2) telemetry.addData("Time took:", autoTime);
        telemetry.addData("currentState", extendArmState);
        telemetry.addData("extendArm1 Power", extendArm1.getPower());
        telemetry.addData("extendArm2 Power", extendArm2.getPower());
        telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
        telemetry.addData("target", extendArmSS.getTarget());
        telemetry.addData("eaCpos1", extendArmSS.getInches1());
        telemetry.addData("eaCpos2", extendArmSS.getInches2());
        telemetry.addData("eaPower", extendArm1.getPower());
        telemetry.addData("error1", Math.abs(extendArmSS.getTarget() - extendArmSS.getInches1()));
        telemetry.addData("error2", Math.abs(extendArmSS.getTarget() - extendArmSS.getInches2()));
        telemetry.addData("errorAvg", (Math.abs(extendArmSS.getTarget() - extendArmSS.getInches1()) + Math.abs(extendArmSS.getTarget() - extendArmSS.getInches2())) / 2);
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
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}

