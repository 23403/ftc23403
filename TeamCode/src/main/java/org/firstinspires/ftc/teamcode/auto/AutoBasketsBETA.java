package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.variables.AutoVariables;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Started code  @  2/16/25  @  1:35 pm
 * Expected to finish code  @  2/26/25
 * It is a 1+3 baskets auto with park. It hangs a preloaded block and then takes the 3 yellow samples from the ground and puts them in the basket.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 1.0, 2/16/25
 */

@Config("Auto Baskets beta")
@Autonomous(name = "Auto Baskets beta", group = ".ftc23403")
public class AutoBasketsBETA extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private static double speed = 0.3;
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
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(preload,true);
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(grabBlock1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(placeBlock1, true);
                    setPathState(3);
                }
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabBlock2, true);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(placeBlock2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(grabBlock3, true);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(placeBlock3, true);
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
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
    private void extendArmMove(int dis1, int dis2, double power) {
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
    private void submersibleArmMove(int dis1, int dis2, double power) {
        DcMotor submersibleArm1 = hardwareMap.dcMotor.get("SubArm1");
        DcMotor submersibleArm2 = hardwareMap.dcMotor.get("SubArm2");
        // stuff
        AutoVariables.saMovements1+=dis1;
        AutoVariables.saMovements1+=dis2;
        submersibleArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        submersibleArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        submersibleArm2.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        submersibleArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        submersibleArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        submersibleArm1.setTargetPosition(dis1);
        submersibleArm2.setTargetPosition(dis2);
        // move moters
        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        submersibleArm1.setPower(power);
        submersibleArm2.setPower(power);
        int saCpos1 = 0;
        int saCpos2 = 0;
        while (submersibleArm1.isBusy() || submersibleArm2.isBusy()) {
            telemetry.addData("SubmersibleArm1Pos:", submersibleArm1.getCurrentPosition());
            telemetry.addData("SubmersibleArm2Pos:", submersibleArm2.getCurrentPosition());
            telemetry.update();
            saCpos1 = submersibleArm1.getCurrentPosition();
            saCpos2 = submersibleArm2.getCurrentPosition();
        }
        // stop
        submersibleArm1.setTargetPosition(saCpos1);
        submersibleArm2.setTargetPosition(saCpos2);
        submersibleArm1.setPower(1);
        submersibleArm2.setPower(1);
        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void claw(double pos) {
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(pos);
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
        hardwareMap.get(IMU.class, "imu").resetYaw();
        AutoVariables.eaMovements1 = 0;
        AutoVariables.eaMovements2 = 0;
        AutoVariables.saMovements1 = 0;
        AutoVariables.saMovements2 = 0;
        // movement
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
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
    }
}

