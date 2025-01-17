package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous(name="Hook Odometry Blue", group="ftc23403")
public class AutoHookPushOdometry extends LinearOpMode {

    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor turnArm;
    private DcMotor extendArm;
    private Servo hangServo;
    private CRServo clawServo;
    private CRServo clawServo1;
    private int eaPOS;

    /**
     * This OpMode offers POV (point-of-view) style TeleOp control for a direct drive robot.
     *
     * In this POV mode, the left joystick (up and down) moves the robot forward and back, and the
     * right joystick (left and right) spins the robot left (counterclockwise) and right (clockwise).
     */
    @Override
    public void runOpMode() {
        Variables variables = new Variables();
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        turnArm = hardwareMap.get(DcMotor.class, "TurnArm");
        extendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
        hangServo = hardwareMap.get(Servo.class, "hook");
        clawServo = hardwareMap.get(CRServo.class, "claw");
        clawServo1 = hardwareMap.get(CRServo.class, "claw1");

        eaPOS = extendArm.getCurrentPosition();
        int ataPOS = turnArm.getCurrentPosition();

        waitForStart();

        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.531, 56.491, Point.CARTESIAN),
                                new Point(29.458, 56.491, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(29.458, 56.491, Point.CARTESIAN),
                                new Point(35.870, 71.567, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-1))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(35.870, 71.567, Point.CARTESIAN),
                                new Point(36.736, 36.217, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-1))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(36.736, 36.217, Point.CARTESIAN),
                                new Point(62.209, 35.350, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(62.209, 35.350, Point.CARTESIAN),
                                new Point(62.383, 23.740, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-1))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(62.383, 23.740, Point.CARTESIAN),
                                new Point(15.076, 24.087, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(15.076, 24.087, Point.CARTESIAN),
                                new Point(62.556, 23.740, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(62.556, 23.740, Point.CARTESIAN),
                                new Point(62.383, 14.383, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-1))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(62.383, 14.383, Point.CARTESIAN),
                                new Point(15.249, 14.556, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(15.249, 14.556, Point.CARTESIAN),
                                new Point(62.729, 13.863, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(62.729, 13.863, Point.CARTESIAN),
                                new Point(62.556, 8.318, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-1))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(62.556, 8.318, Point.CARTESIAN),
                                new Point(7.971, 8.144, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true);
    }


    // movement functions

    private void hangArm(double Pos) {
        hangServo.setPosition(Pos);
    }


    /*
     * Describe this function...
     */
    private void extendArmMoveOld(double power) {
        // E
        extendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendArm.setPower(power);
    }

    private void motorRest(double Time) {
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        int RealTime = (int) Time*1000;
        sleep(RealTime);
    }

    private void turnArmMove(int dis, double power) {
        turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        turnArm.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        // turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        turnArm.setTargetPosition(turnArm.getCurrentPosition() + dis);
        // move moters
        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        turnArm.setPower(power);

        while (turnArm.isBusy()) {
            telemetry.addData("TurnArmPos:", turnArm.getCurrentPosition());
            telemetry.update();
        }
        // stop
        turnArm.setPower(0);
    }

    private void extendArmMove(int dis, double power) {
        extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        extendArm.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        extendArm.setTargetPosition(dis);
        // move moters
        extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        extendArm.setPower(power);

        while (extendArm.isBusy()) {
            telemetry.addData("ExtendArmPos:", extendArm.getCurrentPosition());
            telemetry.update();
        }
        // stop
        extendArm.setPower(0);
    }

    private void claw(double Pow, double Time) {
        clawServo.setPower(-Pow);
        clawServo1.setPower(Pow);
        int RealTime = (int) Time*1000;
        sleep(RealTime);
        clawServo.setPower(0);
        clawServo1.setPower(0);
    }

    private void forwardOld(double power) {
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // power
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
    }


    private void forward(int dis, double power) {
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // reset pos
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        leftBackDrive.setTargetPosition(dis);
        leftFrontDrive.setTargetPosition(dis);
        rightBackDrive.setTargetPosition(dis);
        rightFrontDrive.setTargetPosition(dis);
        // move moters
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);

        while (leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy()  && rightFrontDrive.isBusy() ) {
            telemetry.addData("LeftBackPos:", leftBackDrive.getCurrentPosition());
            telemetry.addData("LeftFrontPos:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightBackPos:", rightBackDrive.getCurrentPosition());
            telemetry.addData("RightFrontPos:", rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }
        // stop
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void turn(int dis, double power) {
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        leftBackDrive.setTargetPosition(dis);
        leftFrontDrive.setTargetPosition(dis);
        rightBackDrive.setTargetPosition(dis);
        rightFrontDrive.setTargetPosition(dis);
        // move moters
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);

        while (leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy()  && rightFrontDrive.isBusy() ) {
            telemetry.addData("LeftBackPos:", leftBackDrive.getCurrentPosition());
            telemetry.addData("LeftFrontPos:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightBackPos:", rightBackDrive.getCurrentPosition());
            telemetry.addData("RightFrontPos:", rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }
        // stop
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void sideways(int dis, double power) {
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // reset pos
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        leftBackDrive.setTargetPosition(dis);
        leftFrontDrive.setTargetPosition(dis);
        rightBackDrive.setTargetPosition(dis);
        rightFrontDrive.setTargetPosition(dis);
        // move moters
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);

        while (leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy()  && rightFrontDrive.isBusy() ) {
            telemetry.addData("LeftBackPos:", leftBackDrive.getCurrentPosition());
            telemetry.addData("LeftFrontPos:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightBackPos:", rightBackDrive.getCurrentPosition());
            telemetry.addData("RightFrontPos:", rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }
        // stop
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void diagonal(double FrontPow, double SidePow) {
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setPower(-SidePow+FrontPow);
        leftFrontDrive.setPower(SidePow+FrontPow);
        rightBackDrive.setPower(SidePow+FrontPow);
        rightFrontDrive.setPower(-SidePow+FrontPow);
    }
}
