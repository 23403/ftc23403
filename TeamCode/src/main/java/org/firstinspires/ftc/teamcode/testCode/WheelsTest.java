package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Wheels Test", group="test_ftc23403")
public class WheelsTest extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    @Override
    public void init() {
        // Hardware mapping
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRear");

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
        // Default to full speed
        double wheelSpeed = 1.0;
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
        telemetry.addData("Left stick X", gamepad1.left_stick_x);
        telemetry.addData("Left stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right stick X", gamepad1.right_stick_x);
        telemetry.addData("FL Power", leftFrontPower);
        telemetry.addData("BL Power", leftBackPower);
        telemetry.addData("FR Power", rightFrontPower);
        telemetry.addData("BR Power", rightBackPower);
        telemetry.addData("FL Current", leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BL Current", leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FR Current", rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BR Current", rightBack.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}