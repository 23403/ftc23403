package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.variables.constants.AutoVariables;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.MetroLib;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Started code  @  2/16/25  @  1:35 pm
 * Expected to finish code  @  2/26/25
 * It is a 1+3 baskets auto with park. It hangs a preloaded block and then takes the 3 yellow samples from the ground and puts them in the basket.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.0, 2/16/25
 */

@Disabled
@Config("Auto Baskets beta")
@Autonomous(name = "Auto Baskets beta", group = "old_ftc23403")
public class AutoBasketsBETA extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    public static double speed = 0.8;
    public static Integer pauses = 1000;
    /** store the state of our auto. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathChains/poses: <https://pedro-path-generator.vercel.app/>
     */

    /** line positions */
    private final Pose startPos = new Pose(9, 87, Math.toRadians(90)); // start Pos
    /** different modes */
    private Path preload;
    /** path name */
    private PathChain grabBlock1, placeBlock1, grabBlock2, placeBlock2, grabBlock3, placeBlock3, park;

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
                new Point(9.000, 87.000, Point.CARTESIAN),
                new Point(23.800, 116.900, Point.CARTESIAN),
                new Point(15.900, 128.200, Point.CARTESIAN)
        ));
        preload.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137));
        /* line2 */
        grabBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(15.900, 128.200, Point.CARTESIAN),
                        new Point(28.600, 131.600, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(-180))
                .build();
        /* line3 */
        placeBlock1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(28.600, 131.600, Point.CARTESIAN),
                        new Point(15.900, 128.200, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(137))
                .build();
        /* line4 */
        grabBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(15.900, 128.200, Point.CARTESIAN),
                        new Point(28.600, 121.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(-180))
                .build();
        /* line5 */
        placeBlock2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(28.600, 121.000, Point.CARTESIAN),
                        new Point(15.900, 128.200, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(137))
                .build();
        /* line6 */
        grabBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(15.900, 128.200, Point.CARTESIAN),
                        new Point(33.700, 129.200, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(-137))
                .build();
        /* line7 */
        placeBlock3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(33.700, 129.200, Point.CARTESIAN),
                        new Point(15.900, 128.200, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-137), Math.toRadians(137))
                .build();
        /* line8 */
        park = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(15.900, 128.200, Point.CARTESIAN),
                        new Point(61.100, 96.300, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(-90))
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
                if(!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(preload,true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(grabBlock1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(placeBlock1, true);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(grabBlock2, true);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(placeBlock2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(grabBlock3, true);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(placeBlock3, true);
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
                    timer.wait(pauses);
                    follower.followPath(park,true);
                    setPathState(8);
                }
                break;
        }
    }

    /**
     * Encoder movements for the arms and sliders,
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    private void extendArmMove(int dis1, int dis2) {
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // stuff
        AutoVariables.eaMovements1+=dis1;
        AutoVariables.eaMovements2+=dis2;
        // PID initialization
        //PIDController controller = new PIDController(0, 0);
        double tickPerRevolution = 751.8 / 180;
        int eaPos1 = extendArm1.getCurrentPosition();
        int eaPos2 = extendArm2.getCurrentPosition();
        // formula
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        //double pid1 = controller.calculate(eaPos1, dis1);
        //double pid2 = controller.calculate(eaPos2, dis2);
        //double power1 = pid1 + ff1;
        // power2 = pid2 + ff2;
        // movement
        //extendArm1.setPower(power1);
        //extendArm2.setPower(power2);
        while (extendArm1.isBusy() || extendArm2.isBusy()) {
            telemetry.addData("ExtendArm1Pos:", extendArm1.getCurrentPosition());
            telemetry.addData("ExtendArm2Pos:", extendArm2.getCurrentPosition());
            telemetry.update();
        }
    }
    // servos
    private void intake(double power, int time) {
        ElapsedTime timer = new ElapsedTime();
        CRServo intake1 = hardwareMap.get(CRServo.class, "intakeL"); // goBilda speed
        CRServo intake2 = hardwareMap.get(CRServo.class, "intakeR"); // goBilda speed
        intake2.setDirection(CRServo.Direction.REVERSE);
        while (timer.milliseconds() < time) {
            intake1.setPower(power);
            intake2.setPower(power);
        }
        intake1.setPower(0);
        intake2.setPower(0);
    }
    private void claw1(double pos) {
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // axon
        claw1.setPosition(pos);
    }
    private void claw2(double pos) {
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // axon
        claw2.setPosition(pos);
    }
    private void wrist1(double pos) {
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // axon
        wrist1.setPosition(pos);
    }
    private void wrist2(double pos) {
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 20kg
        wrist2.setPosition(pos);
    }
    private void sweeper(double pos) {
        Servo sweeper = hardwareMap.get(Servo.class, "sweeper"); // goBilda torque
        sweeper.setDirection(Servo.Direction.REVERSE);
        sweeper.setPosition(pos);
    }
    private void arm(double pos) {
        Servo arm = hardwareMap.get(Servo.class, "arm1"); // axon
        arm.setPosition(pos);
    }
    private void submersibleArm(double pos) {
        Servo submersibleArm = hardwareMap.get(Servo.class, "subArm"); // axon
        submersibleArm.setPosition(pos);
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
        MetroLib.setConstants(MConstants.class);
        Calibrate.Auto.clearEverything();
        hardwareMap.get(IMU.class, TwoWheelConstants.IMU_HardwareMapName).resetYaw();
        AutoVariables.eaMovements1 = 0;
        AutoVariables.eaMovements2 = 0;
        // movement
        claw1(0.4);
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

