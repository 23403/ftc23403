package org.firstinspires.ftc.teamcode.testCode.slides;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public void runOpMode() {
        // hardware
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        PIDController controller = new PIDController(Math.sqrt(P), I, D);
        ExtendArmSS extendArmSS = new ExtendArmSS(extendArm1, extendArm2, controller, K, F, CPR, INCHES_PER_REV, MainV5.eaLimitHigh, MainV5.eaLimitLow, eaCorrection);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        // telemetry
        telemetry.addData("currentState", extendArmSS.getState());
        telemetry.addData("power1", extendArmSS.getPower1());
        telemetry.addData("power2", extendArmSS.getPower2());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // values
                extendArmSS.setEaCorrection(eaCorrection);
                extendArmSS.setLimits(MainV5.eaLimitHigh, MainV5.eaLimitLow);
                // preset
                if (gamepad1.b) {
                    extendArmSS.moveTo(presetPos);
                }
                // update
                extendArmSS.update(gamepad1);
                // telemetry
                telemetry.addData("currentState", extendArmSS.getState());
                telemetry.addData("power1", extendArmSS.getPower1());
                telemetry.addData("power2", extendArmSS.getPower2());
                telemetry.addData("preset", extendArmSS.getTarget());
                telemetry.addData("resetTimer S --> MS", extendArmSS.getResetTimer().seconds() + " --> " + extendArmSS.getResetTimer().milliseconds());
                telemetry.addData("pos1", extendArmSS.getInches1());
                telemetry.addData("pos2", extendArmSS.getInches2());
                telemetry.update();
            }
        }
    }
}
