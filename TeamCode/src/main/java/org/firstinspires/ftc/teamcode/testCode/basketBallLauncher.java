package org.firstinspires.ftc.teamcode.testCode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.variables.ConfigVariables;


@TeleOp(name="Basket Ball", group="test_ftc23403")
public class basketBallLauncher extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                extendArm2.setDirection(DcMotor.Direction.REVERSE);
                if (gamepad1.left_trigger > 0) {
                    extendArm1.setPower(1);
                    extendArm2.setPower(1);
                } else if (gamepad1.right_trigger > 0) {
                    extendArm1.setPower(-1);
                    extendArm2.setPower(-1);
                } else {
                    extendArm1.setPower(0);
                    extendArm2.setPower(0);
                }
            }
        }
    }

}
