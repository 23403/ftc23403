package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides;
import org.firstinspires.ftc.teamcode.variables.constants.AutoVariables;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.MetroLib;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Started code  @  2/16/25  @  10:18 am
 * Expected to finish code  @  3/21/25
 * It is a 1+4 specimen auto with park. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and hang them.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 3.0, 2/16/25
 */

@Config("Auto Hook beta")
@Autonomous(name = "Auto Hook beta", group = ".ftc23403")
public class AutoHookBETA extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    public static double speed = 0.8;
    public static Integer pauses = 1000;
    /** store the state of our auto. */
    private int pathState;
    // servos
    public Servo claw1; // axon
    public Servo claw2; // axon
    public Servo wrist1; // axon
    public Servo wrist2; // 20kg
    public Servo sweeper; // goBilda torque
    public Servo submersibleArm1; // axon
    public Servo arm; // axon
    // servo positions
    public static double wristCpos1 = 0.38;
    // 0.5 low pos
    // 0.38 grab from sa
    // 1 high pos
    public static double clawCpos1 = 0.9;
    // 0.4 is close
    // 0.9 is open
    public static double sweeperCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double wristCpos2 = 0.45;
    // 0.07 low pos
    // 0.45 give to ea
    // 0.5 high pos
    public static double clawCpos2 = 0.5;
    // 0.5 is close
    // 0.55 is grab pos
    // 1 is open pos
    public static double armCpos = 0.72;
    // 0.88 low pos
    // 0.72 grab from sa
    // 0 high pos
    public static double subArmCpos = 1;
    // 0 low pos
    // 0.45 high pos
    // pid
    private PIDController controller;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private int slidesTARGET = 0;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathChains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** line positions */
    private final Pose startPos = new Pose(9, 81, Math.toRadians(0)); // start Pos
    /** different modes */
    private Path preload;
    /** path name */
    private PathChain grabSpecimen2, hangSpecimen2, moveToPushLoc, pushSample1, goToPushSample2, pushSample2, goToPushSample3, pushLastSampleAndGrabSpecimen, hangSpecimen3, grabSpecimen4, hangSpecimen4, grabSpecimen5, hangSpecimen5, park;

    /** create paths */
    public void buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* line1 */
        preload = new Path(new BezierCurve(
                new Point(9.000, 81.000, Point.CARTESIAN),
                new Point(32.000, 70.000, Point.CARTESIAN),
                new Point(40.500, 76.000, Point.CARTESIAN)
        ));
        preload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        /* line2 */
        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(40.500, 76.000, Point.CARTESIAN),
                        new Point(20.000, 22.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line3 */
        hangSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(20.000, 22.000, Point.CARTESIAN),
                        new Point(26.000, 60.000, Point.CARTESIAN),
                        new Point(40.500, 75.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line4 */
        moveToPushLoc = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(40.500, 75.000, Point.CARTESIAN),
                        new Point(22.100, 50.900, Point.CARTESIAN),
                        new Point(31.000, 41.000, Point.CARTESIAN),
                        new Point(69.000, 37.000, Point.CARTESIAN),
                        new Point(56.000, 34.000, Point.CARTESIAN),
                        new Point(60.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line5 */
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(60.000, 25.000, Point.CARTESIAN),
                        new Point(20.000, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line6 */
        goToPushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(20.000, 25.000, Point.CARTESIAN),
                        new Point(71.000, 30.000, Point.CARTESIAN),
                        new Point(60.000, 14.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line7 */
        pushSample2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(60.000, 14.000, Point.CARTESIAN),
                        new Point(20.000, 14.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line8 */
        goToPushSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(20.000, 14.000, Point.CARTESIAN),
                        new Point(59.284, 16.496, Point.CARTESIAN),
                        new Point(60.000, 9.500, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line9 */
        pushLastSampleAndGrabSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(60.000, 9.500, Point.CARTESIAN),
                        new Point(3.000, 9.000, Point.CARTESIAN),
                        new Point(1.000, 8.000, Point.CARTESIAN),
                        new Point(22.000, 26.000, Point.CARTESIAN),
                        new Point(20.000, 22.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line10 */
        hangSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(20.000, 22.000, Point.CARTESIAN),
                        new Point(26.000, 60.000, Point.CARTESIAN),
                        new Point(40.500, 74.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line11 */
        grabSpecimen4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(40.500, 74.000, Point.CARTESIAN),
                        new Point(20.000, 22.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line12 */
        hangSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(20.000, 22.000, Point.CARTESIAN),
                        new Point(26.000, 60.000, Point.CARTESIAN),
                        new Point(40.500, 73.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line13 */
        grabSpecimen5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(40.500, 73.000, Point.CARTESIAN),
                        new Point(20.000, 22.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line14 */
        hangSpecimen5 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(20.000, 22.000, Point.CARTESIAN),
                        new Point(26.000, 60.000, Point.CARTESIAN),
                        new Point(40.500, 72.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        /* line15 */
        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(40.500, 72.000, Point.CARTESIAN),
                        new Point(24.000, 64.000, Point.CARTESIAN),
                        new Point(25.000, 56.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(235))
                .build();
    }

    /* You could check for
        - Follower State: "if(!follower.isBusy() {}"
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
        - Robot Position: "if(follower.getPose().getX() > 36) {}"
    */
    public void autonomousPathUpdate() {
        xyz.nin1275.utils.Timer timer = new xyz.nin1275.utils.Timer();
        switch (pathState) {
            case 0: /* line1 */
                if(follower.getPose().getX() > 39) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700);
                }
                if(!follower.isBusy()) {
                    follower.followPath(preload,true);
                    timer.wait(pauses);
                    claw1(0.9);
                    setPathState(1);
                }
                break;
            case 1: /* line2 */
                extendArmMove(0);
                arm(0.96);
                wrist1(0.65);
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen2,true);
                    timer.wait(pauses);
                    claw1(0.4);
                    arm(0.8);
                    setPathState(2);
                }
                break;
            case 2: /* line3 */
                if(follower.getPose().getX() > 39) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700);
                }
                if (!follower.isBusy()) {
                    follower.followPath(hangSpecimen2, true);
                    timer.wait(pauses);
                    claw1(0.9);
                    extendArmMove(0);
                    setPathState(3);
                    // setPathState(15);
                }
                break;
            case 3: /* line4 */
                extendArmMove(0);
                arm(0.3);
                wrist1(0.45);
                if (!follower.isBusy()) {
                    follower.followPath(moveToPushLoc, true);
                    timer.wait(pauses);
                    setPathState(4);
                }
                break;
            case 4: /* line5 */
                if(!follower.isBusy()) {
                    follower.followPath(pushSample1,true);
                    timer.wait(pauses);
                    setPathState(5);
                }
                break;
            case 5: /* line6 */
                if (!follower.isBusy()) {
                    follower.followPath(goToPushSample2, true);
                    timer.wait(pauses);
                    setPathState(6);
                }
            case 6: /* line7 */
                if (!follower.isBusy()) {
                    follower.followPath(pushSample2, true);
                    timer.wait(pauses);
                    setPathState(7);
                }
                break;
            case 7: /* line8 */
                if(!follower.isBusy()) {
                    follower.followPath(goToPushSample3,true);
                    timer.wait(pauses);
                    setPathState(8);
                }
                break;
            case 8: /* line9 */
                if (!follower.isBusy()) {
                    follower.followPath(pushLastSampleAndGrabSpecimen, true);
                    timer.wait(300);
                    extendArmMove(0);
                    arm(0.96);
                    wrist1(0.65);
                    claw1(0.9);
                    timer.wait(pauses);
                    arm(0.8);
                    claw1(0.4);
                    setPathState(9);
                }
                break;
            case 9: /* line10 */
                if(follower.getPose().getX() > 39) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700);
                }
                if (!follower.isBusy()) {
                    follower.followPath(hangSpecimen3, true);
                    timer.wait(pauses);
                    claw1(0.9);
                    setPathState(10);
                }
                break;
            case 10: /* line11 */
                extendArmMove(0);
                arm(0.96);
                wrist1(0.65);
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen4, true);
                    timer.wait(pauses);
                    arm(0.8);
                    claw1(0.4);
                    setPathState(11);
                }
                break;
            case 11: /* line12 */
                if(follower.getPose().getX() > 39) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700);
                }
                if (!follower.isBusy()) {
                    follower.followPath(hangSpecimen4, true);
                    timer.wait(pauses);
                    claw1(0.9);
                    setPathState(12);
                }
                break;
            case 12: /* line13 */
                extendArmMove(0);
                arm(0.96);
                wrist1(0.65);
                if (!follower.isBusy()) {
                    follower.followPath(grabSpecimen5, true);
                    timer.wait(pauses);
                    arm(0.8);
                    claw1(0.4);
                    setPathState(13);
                }
                break;
            case 13: /* line14 */
                if(follower.getPose().getX() > 39) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700);
                }
                if(!follower.isBusy()) {
                    follower.followPath(hangSpecimen5,true);
                    timer.wait(pauses);
                    claw1(0.9);
                    setPathState(14);
                }
                break;
            case 14: /* line15 */
                extendArmMove(0);
                arm(0.3);
                wrist1(0.45);
                if(follower.getPose().getX() < 28) {
                    submersibleArm(0.45);
                }
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    timer.wait(pauses);
                }
                break;
        }
    }

    /**
     * Encoder movements for the arms and sliders,
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    private void extendArmMove(int pos) {
        slidesTARGET = pos;
    }
    private void extendArmMoveOLD(int dis1, int dis2, double power) {
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // stuff
        AutoVariables.eaMovements1+=dis1;
        AutoVariables.eaMovements2+=dis2;
        // formula
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        extendArm1.setTargetPosition(dis1);
        extendArm2.setTargetPosition(dis2);
        // move moters
        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        int eaCpos1 = 0;
        int eaCpos2 = 0;
        while (extendArm1.isBusy() || extendArm2.isBusy()) {
            telemetry.addData("ExtendArm1Pos:", extendArm1.getCurrentPosition());
            telemetry.addData("ExtendArm2Pos:", extendArm2.getCurrentPosition());
            telemetry.update();
            eaCpos1 = extendArm1.getCurrentPosition();
            eaCpos2 = extendArm2.getCurrentPosition();
        }
        // stop
        extendArm1.setTargetPosition(eaCpos1);
        extendArm2.setTargetPosition(eaCpos2);
        extendArm1.setPower(1);
        extendArm2.setPower(1);
        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // servos
    private void claw1(double pos) {
        clawCpos1 = pos;
    }
    private void claw2(double pos) {
        clawCpos2 = pos;
    }
    private void wrist1(double pos) {
        wristCpos1 = pos;
    }
    private void wrist2(double pos) {
        wristCpos2 = pos;
    }
    private void sweeper(double pos) {
        sweeperCpos = pos;
    }
    private void arm(double pos) {
        armCpos = pos;
    }
    private void submersibleArm(double pos) {
        subArmCpos = pos;
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        follower.setMaxPower(speed);
        autonomousPathUpdate();
        // servos
        claw1.setPosition(clawCpos1);
        claw2.setPosition(clawCpos2);
        wrist1.setPosition(wristCpos1);
        wrist2.setPosition(wristCpos2);
        sweeper.setPosition(sweeperCpos);
        submersibleArm1.setPosition(subArmCpos);
        arm.setPosition(armCpos);
        // extendArm code
        controller.setPID(PIDTuneSlides.P, PIDTuneSlides.I, PIDTuneSlides.D);
        int eaCpos1 = extendArm1.getCurrentPosition();
        int eaCpos2 = extendArm2.getCurrentPosition();
        double pid = controller.calculate(eaCpos1, slidesTARGET);
        double ff = PIDTuneSlides.F;
        double power = pid + ff;
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        // Feedback to Driver Hub
        telemetry.addData("PIDF", "P: " + PIDTuneSlides.P + " I: " + PIDTuneSlides.I + " D: " + PIDTuneSlides.D + " F: " + PIDTuneSlides.F);
        telemetry.addData("target", slidesTARGET);
        telemetry.addData("eaCpos1", eaCpos1);
        telemetry.addData("eaCpos2", eaCpos2);
        telemetry.addData("eaPower", power);
        telemetry.addData("error1", Math.abs(slidesTARGET - eaCpos1));
        telemetry.addData("error2", Math.abs(slidesTARGET - eaCpos2));
        telemetry.addData("errorAvg", (Math.abs(slidesTARGET - eaCpos1) + Math.abs(slidesTARGET - eaCpos2)) / 2);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        // hardware
        MetroLib.setConstants(MConstants.class);
        Calibrate.Auto.clearEverything();
        hardwareMap.get(IMU.class, TwoWheelConstants.IMU_HardwareMapName).resetYaw();
        AutoVariables.eaMovements1 = 0;
        AutoVariables.eaMovements2 = 0;
        controller = new PIDController(PIDTuneSlides.P, PIDTuneSlides.I, PIDTuneSlides.D);
        // motors
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        claw1 = hardwareMap.get(Servo.class, "claw1"); // axon
        claw2 = hardwareMap.get(Servo.class, "claw2"); // axon
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // axon
        wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 20kg
        sweeper = hardwareMap.get(Servo.class, "sweeper"); // goBilda torque
        submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // axon
        arm = hardwareMap.get(Servo.class, "arm"); // axon
        // directions
        sweeper.setDirection(Servo.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // starting pos
        claw1(0.4);
        // movement
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPos);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        Calibrate.Auto.savePositions(List.of(AutoVariables.eaMovements1, AutoVariables.eaMovements2));
        Calibrate.Auto.saveLastKnownPos(follower.getPose());
    }
}

