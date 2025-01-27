// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// 
// @TeleOp(name = "Main1")
// public class COPYME extends LinearOpMode {
//     // Declare motors and servos
//     private DcMotor leftBackDrive, rightFrontDrive, rightBackDrive, leftFrontDrive;
//     private DcMotor turnArm, extendArm;
//     private Servo hangServo;
//     private CRServo clawServo, clawServo1;
//   
//     private int teLimitHigh = -2870;
//     private int teLimitLow = -200;
//     private double wheelSpeed = 1.0;  // Adjust as needed
// 
//     @Override
//     public void runOpMode() {
//         // Initialize hardware
//         leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
//         rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
//         rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
//         leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
//         turnArm = hardwareMap.get(DcMotor.class, "TurnArm");
//         extendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
//         hangServo = hardwareMap.get(Servo.class, "hook");
//         clawServo = hardwareMap.get(CRServo.class, "claw");
//         clawServo1 = hardwareMap.get(CRServo.class, "claw1");
//         
//         // Set directions of motors
//         rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//         rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
// 
//         waitForStart();
// 
//         while (opModeIsActive()) {
//             // Control robot movement
//             double leftPower = (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * wheelSpeed;
//             double rightPower = (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * wheelSpeed;
// 
//             leftBackDrive.setPower(leftPower);
//             leftFrontDrive.setPower(leftPower);
//             rightBackDrive.setPower(rightPower);
//             rightFrontDrive.setPower(rightPower);
// 
//             // Control extend arm with DPad
//             int tePOS = extendArm.getCurrentPosition();
//             if (gamepad1.dpad_up && tePOS > teLimitHigh) {
//                 extendArm.setPower(-0.8);
//             } else if (gamepad1.dpad_down && tePOS < teLimitLow) {
//                 extendArm.setPower(0.8);
//             } else {
//                 extendArm.setPower(0);
//             }
// 
// //
//             if (gamepad1.right_stick_y < 0) {
//                 turnArm.setPower(-0.8);
//             } else if (gamepad1.right_stick_y > 0) {
//                 turnArm.setPower(0.8);
//             } else {
//                 turnArm.setPower(0);
//             }
// 
// 
// 
// if (gamepad1.left_bumper) {
//           clawServo.setPower(1);
//           clawServo1.setPower(-1);
//         } else if (gamepad1.right_bumper){
//           clawServo.setPower(-1);
//           clawServo1.setPower(1);
//         } else {
//           clawServo.setPower(0);
//           clawServo1.setPower(0);
//         }
//         
//           if(gamepad1.y) {
//           hangServo.setPosition(0.4);
//         } else if(gamepad1.a) {
//           hangServo.setPosition(0);
//         }
//         
//             telemetry.addData("Extend Arm Position", tePOS);
//             telemetry.update();
//         }
//     }
// }