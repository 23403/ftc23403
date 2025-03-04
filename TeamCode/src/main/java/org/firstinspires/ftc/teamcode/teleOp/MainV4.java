package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Main v4")
public class MainV4 extends OpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private double wheelSpeed = 1.0; // Default to full speed

    @Override
    public void init() {
        // Hardware mapping
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");

        // Reverse motors if needed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read gamepad inputs
        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x; // Left/Right
        double turn = gamepad1.right_stick_x;  // Rotation

        // Calculate motor power
        double leftFrontPower = (drive + strafe + turn) * wheelSpeed;
        double leftBackPower = (drive - strafe + turn) * wheelSpeed;
        double rightFrontPower = (drive - strafe - turn) * wheelSpeed;
        double rightBackPower = (drive + strafe - turn) * wheelSpeed;

        // Set motor power
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        // Debugging telemetry
        telemetry.addData("Wheel Speed", wheelSpeed);
        telemetry.addData("Joystick X", gamepad1.left_stick_x);
        telemetry.addData("Joystick Y", gamepad1.left_stick_y);
        telemetry.addData("Turn", gamepad1.right_stick_x);
        telemetry.addData("LF Power", leftFrontPower);
        telemetry.addData("LB Power", leftBackPower);
        telemetry.addData("RF Power", rightFrontPower);
        telemetry.addData("RB Power", rightBackPower);
        telemetry.update();
    }
}
