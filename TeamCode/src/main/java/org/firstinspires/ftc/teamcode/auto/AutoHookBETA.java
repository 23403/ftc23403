package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.teleOp.MainV4;
import org.firstinspires.ftc.teamcode.variables.constants.AutoVariables;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.constants.PIDConstants;
import xyz.nin1275.controllers.PIDController;
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
    public Servo submersibleArm; // axon
    public Servo arm; // axon
    public CRServo intake1; // goBilda speed
    public CRServo intake2; // goBilda speed
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
    // corrections


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathChains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** line positions */
    private final Pose startPos = new Pose(8.9, 63.3, Math.toRadians(0)); // start Pos
    /** different modes */
    private Path preload;
    /** path name */
    private PathChain grabSpecimen1, hangSpecimen1, moveToPushLoc, rotate1, rotateBack1, rotate2, moveRotate3, rotate3, grabSpecimen2, hangSpecimen2, grabSpecimen3, hangSpecimen3, grabSpecimen4, hangSpecimen4, park;

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
                new Point(7.800, 56.600, Point.CARTESIAN),
                new Point(16.700, 56.500, Point.CARTESIAN),
                new Point(39.000, 76.000, Point.CARTESIAN)
        ));
        preload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
        /* line2 */
        grabSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(38.000, 76.000, Point.CARTESIAN),
                        new Point(23.000, 28.000, Point.CARTESIAN),
                        new Point(16.500, 31.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        /* line3 */
        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(16.500, 31.000, Point.CARTESIAN),
                        new Point(21.000, 28.000, Point.CARTESIAN),
                        new Point(38.000, 75.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        /* line4 */
        moveToPushLoc = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(38.000, 75.000, Point.CARTESIAN),
                        new Point(30.600, 49.000, Point.CARTESIAN),
                        new Point(38.000, 40.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(138))
                .build();
        /* line5 */
        rotate1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(38.000, 40.000, Point.CARTESIAN),
                        new Point(38.000, 40.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(58))
                .build();
        /* line6 */
        rotateBack1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(38.000, 40.000, Point.CARTESIAN),
                        new Point(38.000, 40.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(58), Math.toRadians(138))
                .build();
        /* line7 */
        rotate2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(38.000, 40.000, Point.CARTESIAN),
                        new Point(38.000, 40.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(58))
                .build();
        /* line8 */
        moveRotate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(38.000, 40.000, Point.CARTESIAN),
                        new Point(38.000, 32.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(58), Math.toRadians(138))
                .build();
        /* line9 */
        rotate3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(38.000, 32.000, Point.CARTESIAN),
                        new Point(38.000, 32.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(58))
                .build();
        /* line10 */
        grabSpecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(38.000, 32.000, Point.CARTESIAN),
                        new Point(16.500, 31.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(58), Math.toRadians(-180))
                .build();
        /* line11 */
        hangSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(16.500, 31.000, Point.CARTESIAN),
                        new Point(21.000, 29.000, Point.CARTESIAN),
                        new Point(38.000, 75.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        /* line12 */
        grabSpecimen3 = follower.pathBuilder()
                .addPath( new BezierCurve(
                        new Point(38.000, 75.000, Point.CARTESIAN),
                        new Point(22.000, 28.000, Point.CARTESIAN),
                        new Point(16.500, 31.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .build();
        /* line13 */
        hangSpecimen3 = follower.pathBuilder()
                .addPath( new BezierCurve(
                        new Point(16.500, 31.000, Point.CARTESIAN),
                        new Point(22.000, 28.000, Point.CARTESIAN),
                        new Point(38.000, 75.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        /* line14 */
        grabSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(38.000, 75.000, Point.CARTESIAN),
                        new Point(21.000, 29.000, Point.CARTESIAN),
                        new Point(16.500, 31.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-180))
                .build();
        /* line15 */
        hangSpecimen4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(16.500, 31.000, Point.CARTESIAN),
                        new Point(22.000, 28.000, Point.CARTESIAN),
                        new Point(38.000, 75.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
                .build();
        /* line16 */
        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(23.000, 42.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
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
            case 0:
                if(follower.getPose().getX() > 38) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700, 2700, 1);
                }
                if(!follower.isBusy()) {
                    follower.followPath(preload,true);
                    timer.wait(pauses);
                    claw1(0.9);
                    setPathState(1);
                }
                break;
            case 1:
                extendArmMove(-2700, -2700, 1);
                arm(0.96);
                wrist1(0.65);
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen1,true);
                    timer.wait(pauses);
                    claw1(0.4);
                    setPathState(2);
                }
                break;
            case 2:
                if(follower.getPose().getX() > 38) {
                    arm(0.35);
                    wrist1(0.7);
                    extendArmMove(2700, 2700, 1);
                }
                if (!follower.isBusy()) {
                    follower.followPath(hangSpecimen1, true);
                    timer.wait(pauses);
                    claw1(0.9);
                    extendArmMove(-2700, -2700, 1);
                    // setPathState(3);
                    setPathState(15);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(moveToPushLoc, true);
                    timer.wait(pauses);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(rotate1,true);
                    timer.wait(pauses);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(rotateBack1, true);
                    timer.wait(pauses);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(rotate2, true);
                    timer.wait(pauses);
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(moveRotate3,true);
                    timer.wait(pauses);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(rotate3, true);
                    timer.wait(pauses);
                    setPathState(9);
                }
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabSpecimen2, true);
                    timer.wait(pauses);
                    setPathState(10);
                }
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(hangSpecimen2, true);
                    timer.wait(pauses);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(grabSpecimen3, true);
                    timer.wait(pauses);
                    setPathState(12);
                }
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(hangSpecimen3, true);
                    timer.wait(pauses);
                    setPathState(13);
                }
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(grabSpecimen4,true);
                    timer.wait(pauses);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(hangSpecimen4, true);
                    timer.wait(pauses);
                    setPathState(15);
                }
            case 15:
                subArmCpos = 0.45;
                if (!follower.isBusy()) {
                    follower.followPath(park, true);
                    setPathState(16);
                }
        }
    }

    /**
     * Encoder movements for the arms and sliders,
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    private void extendArmMovePID(int dis1, int dis2, double power) {
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        // stuff
        AutoVariables.eaMovements1+=dis1;
        AutoVariables.eaMovements2+=dis2;
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    private void extendArmMove(int dis1, int dis2, double power) {
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
    private void intake(double power, int time) {
        ElapsedTime timer = new ElapsedTime();
        intake2.setPower(0);
    }
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
        submersibleArm.setPosition(subArmCpos);
        arm.setPosition(armCpos);
        // Feedback to Driver Hub
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
        Constants.setConstants(FConstants.class, LConstants.class);
        MetroLib.setConstants(MConstants.class);
        Calibrate.Auto.clearEverything();
        hardwareMap.get(IMU.class, TwoWheelConstants.IMU_HardwareMapName).resetYaw();
        AutoVariables.eaMovements1 = 0;
        AutoVariables.eaMovements2 = 0;
        // servos
        claw1 = hardwareMap.get(Servo.class, "claw1"); // axon
        claw2 = hardwareMap.get(Servo.class, "claw2"); // axon
        wrist1 = hardwareMap.get(Servo.class, "wrist1"); // axon
        wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 20kg
        sweeper = hardwareMap.get(Servo.class, "sweeper"); // goBilda torque
        submersibleArm = hardwareMap.get(Servo.class, "subArm"); // axon
        arm = hardwareMap.get(Servo.class, "arm"); // axon
        intake1 = hardwareMap.get(CRServo.class, "intakeL"); // goBilda speed
        intake2 = hardwareMap.get(CRServo.class, "intakeR"); // goBilda speed
        // directions
        intake2.setDirection(CRServo.Direction.REVERSE);
        sweeper.setDirection(Servo.Direction.REVERSE);
        // starting pos
        claw1(0.4);
        // movement
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = new Follower(hardwareMap);
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

