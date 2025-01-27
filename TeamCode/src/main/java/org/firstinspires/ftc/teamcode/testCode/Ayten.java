package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Ayten code")
@Disabled
public class Ayten extends LinearOpMode {

    private DcMotor leftBackDrive, rightFrontDrive, rightBackDrive, leftFrontDrive;
    private DcMotor turnArm, extendArm;
    private Servo hangServo;
    private CRServo clawServo, clawServo1;

    private int MAX_SLIDER_POSITION = -2200; //upp
    private int MIN_SLIDER_POSITION = -20; //down


    //private int MAX_UP_DOWN_POSITION = -100 ;//uppp
    // private int MIN_UP_DOWN_POSITION = 2500;//Downnnn

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        turnArm = hardwareMap.get(DcMotor.class, "TurnArm");
        extendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
        hangServo = hardwareMap.get(Servo.class, "hook");
        clawServo = hardwareMap.get(CRServo.class, "claw");
        clawServo1 = hardwareMap.get(CRServo.class, "claw1");

        // Set directions of motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);



        // Variables for control states
        float vertical, horizontal, pivot;

        waitForStart();

        while (opModeIsActive()) {
            // Joystick input
            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;

            // Drivetrain control

            rightFrontDrive.setPower(-pivot + (vertical - horizontal));
            rightBackDrive.setPower(-pivot + vertical + horizontal);
            leftFrontDrive.setPower(pivot + vertical + horizontal);
            leftBackDrive.setPower(pivot + (vertical - horizontal));

            // Slider motor control
            int sliderPosition = extendArm.getCurrentPosition();
            if (gamepad1.dpad_up && sliderPosition > MAX_SLIDER_POSITION) {
                extendArm.setPower(-0.8);
            } else if (gamepad1.dpad_down && sliderPosition < MIN_SLIDER_POSITION) {
                extendArm.setPower(0.8);
            } else {
                extendArm.setPower(0);
            }

            // Up/down motor control
            //int upDownPosition = Up_down.getCurrentPosition();
            if (gamepad1.dpad_left) {
                turnArm.setPower(0.8);
            } else if (gamepad1.dpad_right) {
                turnArm.setPower(-0.8);
            } else {
                turnArm.setPower(0);
            }

            if (gamepad1.left_bumper) {
                clawServo.setPower(1);
                clawServo1.setPower(-1);
            } else if (gamepad1.right_bumper){
                clawServo.setPower(-1);
                clawServo1.setPower(1);
            } else {
                clawServo.setPower(0);
                clawServo1.setPower(0);
            }

            if(gamepad1.y) {
                hangServo.setPosition(0.4);
            } else if(gamepad1.a) {
                hangServo.setPosition(0);
            }


            // Update telemetry

            telemetry.addData("Extend Position", sliderPosition);
            telemetry.update();
        }
    }
}