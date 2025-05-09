package org.firstinspires.ftc.teamcode.auto.old;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * Code Conductor's first auto with odometry.
 * Started code  @  2/6/25  @  5:37 pm
 * Finished code  @  2/7/25  @  10:49 pm
 * 2 hours each day
 * It is a 1+0 specimen auto. It hangs a preloaded specimen and then push the 3 samples from the ground and park.
 * @author David Grieas - 23403 Code Conductors
 * @version 1.2, 2/7/25
 */

@Disabled
@Autonomous(name = "Auto Hook", group = "old_ftc23403")
public class AutoHookOdometry extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;

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
    private final Pose startPos = new Pose(9, 56, Math.toRadians(0)); // start Pos
    private final Pose moveForward = new Pose(29, 56, Math.toRadians(0)); // line1
    private final Pose specimenLoc = new Pose(33, 71, Math.toRadians(0)); // line2
    private final Pose specimenLoc1 = new Pose(30, 71, Math.toRadians(0)); // line2a
    private final Pose moveToSide = new Pose(35, 36, Math.toRadians(0)); // line3
    private final Pose moveForward1 = new Pose(62, 36, Math.toRadians(0)); // line4
    private final Pose moveSide = new Pose(62, 28, Math.toRadians(0)); // line5
    private final Pose pushBott = new Pose(15, 25, Math.toRadians(0)); // line6
    private final Pose goBack = new Pose(62, 23, Math.toRadians(0)); // line7
    private final Pose moveSide1 = new Pose(62, 15.5, Math.toRadians(0)); // line8
    private final Pose pushBott1 = new Pose(15, 17, Math.toRadians(0)); // line9
    private final Pose goBack1 = new Pose(62, 16, Math.toRadians(0)); // line10
    private final Pose moveSide2 = new Pose(62, 14.5, Math.toRadians(0)); // line11
    private final Pose pushBott2 = new Pose(16, 13.5, Math.toRadians(0)); // line12
    /** different modes */
    private Path scorePreload;
    /** path name */
    private PathChain hook, hook1, goYellow, pushBot1, pushBot2, pushBot3, pushBot4, pushBot5, pushBot6, pushBot7, pushBot8, pushBot9;

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

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPos), new Point(moveForward)));
        scorePreload.setLinearHeadingInterpolation(startPos.getHeading(), moveForward.getHeading());
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        hook = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveForward), new Point(specimenLoc)))
                .setLinearHeadingInterpolation(moveForward.getHeading(), specimenLoc.getHeading())
                .build();
        hook1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenLoc), new Point(specimenLoc1)))
                .setLinearHeadingInterpolation(specimenLoc.getHeading(), specimenLoc1.getHeading())
                .build();
        goYellow = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenLoc1), new Point(moveToSide)))
                .setLinearHeadingInterpolation(specimenLoc1.getHeading(), moveToSide.getHeading())
                .build();
        pushBot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveToSide), new Point(moveForward1)))
                .setLinearHeadingInterpolation(moveToSide.getHeading(), moveForward1.getHeading())
                .build();
        pushBot2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveForward1), new Point(moveSide)))
                .setLinearHeadingInterpolation(moveForward1.getHeading(), moveSide.getHeading())
                .build();
        pushBot3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveSide), new Point(pushBott)))
                .setLinearHeadingInterpolation(moveSide.getHeading(), pushBott.getHeading())
                .build();
        pushBot4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushBott), new Point(goBack)))
                .setLinearHeadingInterpolation(pushBott.getHeading(), goBack.getHeading())
                .build();
        pushBot5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goBack), new Point(moveSide1)))
                .setLinearHeadingInterpolation(goBack.getHeading(), moveSide1.getHeading())
                .build();
        pushBot6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveSide1), new Point(pushBott1)))
                .setLinearHeadingInterpolation(moveSide1.getHeading(), pushBott1.getHeading())
                .build();
        pushBot7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushBott1), new Point(goBack1)))
                .setLinearHeadingInterpolation(pushBott1.getHeading(), goBack1.getHeading())
                .build();
        pushBot8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(goBack1), new Point(moveSide2)))
                .setLinearHeadingInterpolation(goBack1.getHeading(), moveSide2.getHeading())
                .build();
        pushBot9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(moveSide2), new Point(pushBott2)))
                .setLinearHeadingInterpolation(moveSide2.getHeading(), pushBott2.getHeading())
                .build();
    }

    /* You could check for
        - Follower State: "if(!follower.isBusy() {}"
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
        - Robot Position: "if(follower.getPose().getX() > 36) {}"
    */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                turnArmMove(775, 1);
                extendArmMove(451, 447, 1);
                if(!follower.isBusy()) {
                    follower.followPath(hook,true);
                    setPathState(2);
                }
                break;
            case 2:
                wait(30);
                claw(0.47);
                if (!follower.isBusy()) {
                    follower.followPath(hook1, true);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(goYellow,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot4, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot5,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot6,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot7,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot8,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(pushBot9,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    // movements
    private void turnArmMove(int dis, double power) {
        DcMotor turnArm = hardwareMap.dcMotor.get("TurnArm");
        turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // reset pos
        // turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        turnArm.setTargetPosition(turnArm.getCurrentPosition() + dis);
        // move moters
        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        turnArm.setPower(power);
        int taCpos = 0;
        while (turnArm.isBusy()) {
            telemetry.addData("TurnArmPos:", turnArm.getCurrentPosition());
            telemetry.update();
            taCpos = turnArm.getCurrentPosition();
        }
        // stop
        turnArm.setTargetPosition(taCpos);
        turnArm.setPower(1);
        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void extendArmMove(int dis1, int dis2, double power) {
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        // stuff
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
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
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
        hardwareMap.get(GoBildaPinpointDriver.class, PinpointConstants.hardwareMapName).resetPosAndIMU();
        // servos
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        // formulas
        claw.setDirection(Servo.Direction.REVERSE);
        // positions
        wrist.setPosition(1);
        claw.setPosition(0.48);
        // arm goes from 180 to 992
        turnArmMove(180, 1);
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
    }
}

