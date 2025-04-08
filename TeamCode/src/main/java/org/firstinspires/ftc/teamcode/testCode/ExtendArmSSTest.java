package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSS;

@Config("ExtendArmSS Test")
@TeleOp(name="ExtendArmSS Test", group="test_ftc23403")
public class ExtendArmSSTest extends LinearOpMode {
    public static boolean eaCorrection = true;
    public static double presetPos = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        ExtendArmSS extendArmSS = new ExtendArmSS(extendArm1, extendArm2);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                extendArmSS.setEaCorrection(eaCorrection);
                extendArmSS.update();
                extendArmSS.moveSlides(gamepad1.dpad_up, gamepad1.dpad_down);
                if (gamepad1.b) {
                    extendArmSS.preset(presetPos);
                }
                telemetry.addData("currentState", extendArmSS.getCurrentState());
                telemetry.update();
            }
        }
    }
}
