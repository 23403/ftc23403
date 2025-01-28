package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.variables.VariablesOld;

@Autonomous(name="Hook Push", group="ftc23403")
public class AutoHookPush extends LinearOpMode {

    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor turnArm;
    private DcMotor extendArm;
    private Servo hangServo;
    private CRServo clawServo;
    private CRServo clawServo1;

    //Variables
    private int taLimitHigh;
    private int taLimitLow;
    private double wheelSpeed;
    private double extendArmSpeed;
    private int taPL;
    private int taPLM;
    private int taPLL;
    private boolean pl;
    private int LowExtendLimit;
    private int HighExtendLimit;
    private double Cir;
    /**
     * This OpMode offers POV (point-of-view) style TeleOp control for a direct drive robot.
     *
     * In this POV mode, the left joystick (up and down) moves the robot forward and back, and the
     * right joystick (left and right) spins the robot left (counterclockwise) and right (clockwise).
     */
    @Override
    public void runOpMode() {
        VariablesOld variables = new VariablesOld();
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        turnArm = hardwareMap.get(DcMotor.class, "TurnArm");
        extendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
        hangServo = hardwareMap.get(Servo.class, "hook");
        clawServo = hardwareMap.get(CRServo.class, "claw");
        clawServo1 = hardwareMap.get(CRServo.class, "claw1");

        //Turn Arm Limits
        taLimitHigh = variables.taLimitHigh;
        taLimitLow = variables.taLimitLow;
        //
        wheelSpeed = variables.wheelSpeed;
        extendArmSpeed = variables.extendArmSpeed;
        LowExtendLimit = extendArm.getCurrentPosition();
        HighExtendLimit = LowExtendLimit + variables.EaMaxConstant;
        // pickup low POS
        taPL = variables.taPL;
        taPLM = variables.taPLM;
        taPLL = variables.taPLL;
        pl = variables.pl;
        int ataPOS = turnArm.getCurrentPosition();

        waitForStart();

        // blue alience code
    /* // raise arms
    turnArmMove(1000, 1);
    extendArmMove(200, 1);
    */
        forward(90, 1);
        // arm
        turnArmMove(1750, 0.7);
        // move
        sideways(160, 1);
        forward(200, 1);
        // hang specimen
        extendArmMove(1300, 0.7);
        extendArmMoveOld(-1);
        turnArmMove(-800, 0.7);
        sleep(500);
        // go to the sides
        forward(-170, 0.3);
        motorRest(0.5);
        // sideways(-550, 1); // to push blocks
        sideways(-700, 0.7); // to park
        motorRest(0.4);
        // forward(350, 1); // to push blocks
        motorRest(0.8);
        // sideways(-200, 1);
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
        rightFrontDrive.setPower(power - 0.05);

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
        leftBackDrive.setPower(-SidePow+FrontPow);
        leftFrontDrive.setPower(SidePow+FrontPow);
        rightBackDrive.setPower(SidePow+FrontPow);
        rightFrontDrive.setPower(-SidePow+FrontPow);
    }
}
