package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Wheels test", group="test_ftc23403")
public class WheelsTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
                DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
                DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
                DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
                leftBackDrive.setPower(1);
                rightFrontDrive.setPower(1);
                rightBackDrive.setPower(1);
                leftFrontDrive.setPower(1);
            }
        }
    }

}