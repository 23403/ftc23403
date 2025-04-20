package org.firstinspires.ftc.teamcode.testCode.slides;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.variables.ConfigVariables;

@Disabled
@TeleOp(name="Sliders test", group="test_ftc23403")
public class SlidersCode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
        ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int eaCpos1 = ConfigVariables.eaCpos1;
                int eaCpos2 = ConfigVariables.eaCpos2;
                int eaLimitLow1 = ConfigVariables.eaLimitLow1;
                int eaLimitLow2 = ConfigVariables.eaLimitLow2;
                extendArm2.setDirection(DcMotor.Direction.REVERSE);
                if (gamepad2.dpad_up) {
                    extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendArm1.setPower(1);
                    extendArm2.setPower(1);
                    ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
                    ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
                } else if (gamepad2.dpad_down) {
                    extendArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendArm1.setPower(-1);
                    extendArm2.setPower(-1);
                    ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
                    ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
                } else {
                    extendArm1.setTargetPosition(eaCpos1);
                    extendArm2.setTargetPosition(eaCpos2);
                    extendArm1.setPower(1);
                    extendArm2.setPower(1);
                    extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
        }
    }

}
