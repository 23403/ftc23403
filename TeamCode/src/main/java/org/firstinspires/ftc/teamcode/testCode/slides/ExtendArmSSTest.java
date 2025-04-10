package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.ExtendArmSS;
import org.firstinspires.ftc.teamcode.teleOp.MainV5;

@Config("ExtendArmSS Test")
@TeleOp(name="ExtendArmSS Test", group="test_ftc23403")
public class ExtendArmSSTest extends LinearOpMode {
    // vars
    public static boolean eaCorrection = true;
    public static double presetPos = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        ExtendArmSS extendArmSS = new ExtendArmSS(extendArm1, extendArm2);
        // reset slides
        extendArmSS.resetSlidesINIT();
        // telemetry
        telemetry.addData("currentState", extendArmSS.getCurrentState());
        telemetry.addData("raw power", extendArmSS.getRawPower());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // values
                extendArmSS.setEaCorrection(eaCorrection);
                extendArmSS.setLimits(MainV5.eaLimitHigh, MainV5.eaLimitLow);
                // preset
                if (gamepad1.b) {
                    extendArmSS.preset(presetPos);
                }
                // update
                extendArmSS.update(gamepad1.dpad_up, gamepad1.dpad_down);
                // telemetry
                telemetry.addData("currentState", extendArmSS.getCurrentState());
                telemetry.addData("raw power", extendArmSS.getRawPower());
                telemetry.addData("preset", extendArmSS.getTargetInches());
                telemetry.addData("syncError", extendArmSS.getSyncError());
                telemetry.addData("resetTimer S --> MS", extendArmSS.getResetTimer().seconds() + " --> " + extendArmSS.getResetTimer().milliseconds());
                telemetry.addData("pos1", extendArmSS.ticksToInches(extendArm1.getCurrentPosition()));
                telemetry.addData("pos2", extendArmSS.ticksToInches(extendArm2.getCurrentPosition()));
                telemetry.update();
            }
        }
    }
}
