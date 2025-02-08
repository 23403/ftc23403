package org.firstinspires.ftc.teamcode.auto;


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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.variables.ConfigVariables;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Hook Odometry", group = ".ftc23403")
public class AutoHookOdometry extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private String pathState;
    private final Pose line0 = new Pose(9, 48, Math.toRadians(270));
    private final Pose line1 = new Pose(42.454873646209386, 70.87364620938628, Math.toRadians(270));
    private Path scorePreload, park;
    private PathChain grabPickup1, scorePickup1, startPush1, push1, startPush2, push2, grabPickup2, grabPickup3, grabPickup4, scorePickup2, scorePickup3, scorePickup4, grabPickup5, scorePickup5;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(line0), new Point(line1)));
        scorePreload.setLinearHeadingInterpolation(line0.getHeading(), line1.getHeading());
    }
    public void autonomousPathUpdate() throws InterruptedException {
        // servos
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        CRServo intake1 = hardwareMap.get(CRServo.class, "intakeL");
        CRServo intake2 = hardwareMap.get(CRServo.class, "intakeR");
        // wrist.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        // starting pos
        switch (pathState) {
            case "ScorePreload":
                wait(300.0);
                turnArmMove(720, 1);
                follower.setMaxPower(1);
                follower.followPath(scorePreload, true);
                setPathState("ReleasePreload");
                break;

            case "ReleasePreload":
                if(!follower.isBusy()){
                    setPathState("PickUp1");
                }
                break;
            case "PickUp1":
                if(!follower.isBusy()) {
                    wait(300);
                }
                break;
            case "ReleasePickUp1":
                if(!follower.isBusy()){
                    wait(300);
                    setPathState("Push1");
                }
                break;
            case "Push1":
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(push1,true);
                    setPathState("Push2.1");
                }

                break;
            case "Push2.1":
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(startPush2,true);
                    setPathState("Push2.2");
                }

                break;
            case "Push2.2":
                if(!follower.isBusy()){
                    follower.setMaxPower(1);
                    follower.followPath(push2,true);
                    setPathState("PickUp2Initiate");
                }
                break;
            case "PickUp2Initiate":
                if(!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup2,true);
                    setPathState("PickUp2");
                }
                break;
            case "PickUp2":
                if(!follower.isBusy()) {
                    wait(300.0);
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup2,true);
                    setPathState("ReleasePickUp2");
                }
                break;
            case "ReleasePickUp2":
                if(!follower.isBusy()){
                    wait(300.0);
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup3,true);
                    setPathState("PickUp3");
                }
                break;
            case "PickUp3":
                if(!follower.isBusy()) {
                    wait(300.0);
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup3,true);
                    setPathState("ReleasePickUp3");
                }
                break;
            case "ReleasePickUp3":
                if(!follower.isBusy()){
                    wait(300.0);
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup4,true);
                    setPathState("PickUp4");
                }
                break;
            case "PickUp4":
                if(!follower.isBusy()) {
                    wait(300.0);
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup4,true);
                    setPathState("ReleasePickUp4");
                }
                break;
            case "ReleasePickUp4":
                if(!follower.isBusy()){
                    wait(300.0);
                    follower.setMaxPower(1);
                    follower.followPath(park,true);
                    setPathState("end");
                }
                break;
        }
    }
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

    private void extendArmMove(int dis, double power) {
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
        extendArm1.setTargetPosition(dis);
        extendArm2.setTargetPosition(dis);
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



    public void setPathState(String pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void wait(double milliseconds) {
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < milliseconds) {
            // Do nothing, just wait
        }
    }
    public void wait(double milliseconds, Path P) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            follower.followPath(P,true);
        }
    }


    @Override
    public void loop() {
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Time", time);
        telemetry.update();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(line0);
        buildPaths();
    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState("ScorePreload");
    }


    @Override
    public void stop() {
        // PersistentPoseStorage.lastKnownPose = follower.getPose();
    }
}