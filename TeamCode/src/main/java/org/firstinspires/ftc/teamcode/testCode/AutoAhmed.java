package org.firstinspires.ftc.teamcode.testCode;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "Auto", group = "ftc23403")
public class AutoAhmed extends OpMode {
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private String pathState;
    private final Pose startPose = new Pose(9, 48, Math.toRadians(270));
    private final Pose scorePose = new Pose(38, 67, Math.toRadians(180));
    private final Pose scoreControl = new Pose(30, 40, Math.toRadians(180));
    private final Pose pickupPose = new Pose(20, 37, Math.toRadians(180));
    private final Pose push1Pose = new Pose(60, 25, Math.toRadians(270));
    private final Pose control1 = new Pose(0, 65, Math.toRadians(270));
    private final Pose push1Pose2 = new Pose(14, 25, Math.toRadians(270));
    private final Pose push2Pose = new Pose(50, 12, Math.toRadians(270));
    private final Pose control2 = new Pose(65, 35, Math.toRadians(270));
    private final Pose push2Pose2 = new Pose(14, 12, Math.toRadians(270));
    private final Pose PickUpControl = new Pose(60, 40, Math.toRadians(180));
    private final Pose parkPose = new Pose(9, 40, Math.toRadians(0));
    private Path scorePreload, park;
    private PathChain grabPickup1, scorePickup1, startPush1, push1, startPush2, push2, grabPickup2, grabPickup3, grabPickup4, scorePickup2, scorePickup3, scorePickup4, grabPickup5, scorePickup5;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));

        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose),new Point(scoreControl), new Point(pickupPose)))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();

        startPush1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(control1), new Point(push1Pose)))
                .setConstantHeadingInterpolation(push1Pose.getHeading())
                .build();
        push1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push1Pose), new Point(push1Pose2)))
                .setConstantHeadingInterpolation(push1Pose2.getHeading())
                .build();

        startPush2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push1Pose2), new Point(control2), new Point(push2Pose)))
                .setConstantHeadingInterpolation(push2Pose.getHeading())
                .build();
        push2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push2Pose), new Point(push2Pose2)))
                .setConstantHeadingInterpolation(push2Pose2.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                //.addPath(new BezierCurve(new Point(push3Pose2), new Point(PickUpControl), new Point(pickupPose.getX() - 2, pickupPose.getY())))
                .addPath(new BezierCurve(new Point(push2Pose2), new Point(PickUpControl), new Point(pickupPose)))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickupPose)))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();

        scorePickup5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(scorePose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }


    public void autonomousPathUpdate() {
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo twist = hardwareMap.servo.get("twist");
        Servo spin = hardwareMap.servo.get("spin");
        Servo grip = hardwareMap.servo.get("grip");

        int ap1 = 2200;
        int sp1 = 0;
        double tp1 = 0.75;

        int ap2 = 1800;
        int sp2 = -225;
        double tp2 = 0.5;

        int ap3 = 550;
        int sp3 = 0;
        double tp3 = 0.9;


        spin.setPosition(0);


        switch (pathState) {
            case "ScorePreload":
                grip.setPosition(0);


                wait(300.0);

                arm.setTargetPosition(ap1);
                slide.setTargetPosition(sp1);
                twist.setPosition(tp1);

                arm.setPower(1);
                slide.setPower(1);

                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                follower.setMaxPower(1);
                follower.followPath(scorePreload, true);
                setPathState("ReleasePreload");
                break;

            case "ReleasePreload":
                if(!follower.isBusy() && arm.getCurrentPosition() >= 2000){

                    arm.setTargetPosition(ap2);
                    slide.setTargetPosition(sp2);
                    twist.setPosition(tp2);


                    wait(300.0);
                    grip.setPosition(1);

                    follower.setMaxPower(1);

                    follower.followPath(grabPickup1,true);


                    arm.setTargetPosition(ap3);
                    slide.setTargetPosition(sp3);
                    twist.setPosition(tp3);

                    setPathState("PickUp1");
                }
                break;
            case "PickUp1":
                if(!follower.isBusy()) {
                    arm.setTargetPosition(ap3);

                    grip.setPosition(0);

                    wait(300.0);

                    arm.setTargetPosition(ap1);
                    slide.setTargetPosition(sp1-50);
                    twist.setPosition(tp1);


                    follower.setMaxPower(1);
                    follower.setMaxPower(1);

                    follower.followPath(scorePickup1,true);
                    setPathState("ReleasePickUp1");
                }
                break;
            case "ReleasePickUp1":
                if(!follower.isBusy() && arm.getCurrentPosition() >= 2000){

                    arm.setTargetPosition(ap2);
                    slide.setTargetPosition(sp2);
                    twist.setPosition(tp2);

                    wait(300.0);

                    grip.setPosition(1);

                    follower.setMaxPower(1);

                    follower.followPath(startPush1,true);

                    arm.setTargetPosition(2000);
                    slide.setTargetPosition(0);
                    twist.setPosition(1);

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
            /*case "Push3.1":
                if(!follower.isBusy()){
                    follower.setMaxPower(1);

                    follower.followPath(startPush3,true);
                    setPathState("Push3.2");
                }

                break;
            case "Push3.2":
                if(!follower.isBusy()){
                    follower.setMaxPower(1);

                    follower.followPath(push3,true);
                    setPathState("PickUp2Initiate");
                }
*/
                break;
            case "PickUp2Initiate":
                if(!follower.isBusy()) {


                    follower.setMaxPower(1);
                    arm.setTargetPosition(ap3);
                    slide.setTargetPosition(sp3);
                    twist.setPosition(tp3);


                    follower.followPath(grabPickup2,true);
                    setPathState("PickUp2");
                }
                break;
            case "PickUp2":
                if(!follower.isBusy()) {


                    grip.setPosition(0);

                    wait(300.0);

                    arm.setTargetPosition(ap1);
                    slide.setTargetPosition(sp1);
                    twist.setPosition(tp1);



                    follower.setMaxPower(1);

                    follower.followPath(scorePickup2,true);
                    setPathState("ReleasePickUp2");
                }
                break;
            case "ReleasePickUp2":
                if(!follower.isBusy() && arm.getCurrentPosition() >= 2000){

                    arm.setTargetPosition(ap2);
                    slide.setTargetPosition(sp2);
                    twist.setPosition(tp2);

                    wait(300.0);

                    grip.setPosition(1);

                    follower.setMaxPower(1);

                    follower.followPath(grabPickup3,true);

                    arm.setTargetPosition(ap3);
                    slide.setTargetPosition(sp3);
                    twist.setPosition(tp3);

                    setPathState("PickUp3");
                }
                break;
            case "PickUp3":
                if(!follower.isBusy()) {


                    grip.setPosition(0);

                    wait(300.0);

                    arm.setTargetPosition(ap1);
                    slide.setTargetPosition(sp1);
                    twist.setPosition(tp1);



                    follower.setMaxPower(1);

                    follower.followPath(scorePickup3,true);
                    setPathState("ReleasePickUp3");
                }
                break;
            case "ReleasePickUp3":
                if(!follower.isBusy() && arm.getCurrentPosition() >= 2000){

                    arm.setTargetPosition(ap2);
                    slide.setTargetPosition(sp2);
                    twist.setPosition(tp2);

                    wait(300.0);

                    grip.setPosition(1);

                    follower.setMaxPower(1);

                    follower.followPath(grabPickup4,true);

                    arm.setTargetPosition(ap3);
                    slide.setTargetPosition(sp3);
                    twist.setPosition(tp3);

                    setPathState("PickUp4");
                }
                break;
            case "PickUp4":
                if(!follower.isBusy()) {


                    grip.setPosition(0);

                    wait(300.0);

                    arm.setTargetPosition(ap1);
                    slide.setTargetPosition(sp1);
                    twist.setPosition(tp1);



                    follower.setMaxPower(1);

                    follower.followPath(scorePickup4,true);
                    setPathState("ReleasePickUp4");
                }
                break;
            case "ReleasePickUp4":
                if(!follower.isBusy() && arm.getCurrentPosition() >= 2000){

                    arm.setTargetPosition(ap2);
                    slide.setTargetPosition(sp2);
                    twist.setPosition(tp2);

                    wait(300.0);

                    grip.setPosition(1);

                    follower.setMaxPower(1);

                    follower.followPath(park,true);

                    arm.setTargetPosition(0);
                    slide.setTargetPosition(0);
                    twist.setPosition(1);

                    setPathState("end");
                }
                break;




        }





    }
    public void setArmToPos(int sPos, int aPos, double tPos){
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        Servo twist = hardwareMap.servo.get("twist");



        arm.setTargetPosition(aPos);
        slide.setTargetPosition(sPos);
        twist.setPosition(tPos);

        arm.setPower(1);
        slide.setPower(1);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        autonomousPathUpdate();


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


        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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