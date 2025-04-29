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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.variables.enums.ExtendArmStates;

import java.util.List;

import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

@Config("PID Tuned Slides Test")
@TeleOp(name="PID Tuned Slides Test", group="test_ftc23403")
public class PIDTunedSlidesTest extends LinearOpMode {
    public static double slidesTARGET = 5;
    public static double eaLimitHigh = 36;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    private static ExtendArmStates extendArmState = ExtendArmStates.FLOATING;
    ElapsedTime resetTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        PIDController controller = new PIDController(Math.sqrt(P), I, D);
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            extendArm1.setPower(-0.4);
            extendArm2.setPower(-0.4);
            telemetry.addLine("RESETTING 0 POS!");
            telemetry.update();
        }
        extendArm1.setPower(0);
        extendArm2.setPower(0);
        Motors.resetEncoders(extendArm1, extendArm2);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
        resetTimer.reset();
        // telemetry.addData("currentState", extendArmSS.getCurrentState());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // extendArm code
                controller.setPID(Math.sqrt(P), I, D);
                // Get current positions
                int eaTicks1 = extendArm1.getCurrentPosition();
                int eaTicks2 = extendArm2.getCurrentPosition();
                // Convert ticks to inches
                double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
                double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
                // vars
                double ff = eaCorrection ? F : 0;
                // controls
                if (gamepad2.dpad_up && eaInches1 < eaLimitHigh) {
                    double pid = controller.calculate(eaInches1, eaLimitHigh);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
                } else if (gamepad2.dpad_down && eaInches1 > eaLimitLow) {
                    double pid = controller.calculate(eaInches1, eaLimitLow);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = ExtendArmStates.MANUAL_MOVEMENT;
                } else if (Math.abs(eaInches1 - eaLimitLow) > 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                    extendArm1.setPower(ff);
                    extendArm2.setPower(ff);
                    if (extendArmState == ExtendArmStates.PRESET_REACHED) Timer.wait(500);
                    extendArmState = eaCorrection ? ExtendArmStates.FORCE_FEED_BACK : ExtendArmStates.FLOATING;
                }
                // states
                if (Math.abs(eaInches1 - eaLimitHigh) < 1 && extendArmState != ExtendArmStates.MOVING_TO_PRESET) {
                    extendArmState = ExtendArmStates.MAX_POS;
                } else if (Math.abs(eaInches1 - eaLimitLow) < 2 && extendArmState != ExtendArmStates.MOVING_TO_PRESET && extendArmState != ExtendArmStates.RESETTING_ZERO_POS && extendArmState != ExtendArmStates.ZERO_POS_RESET && extendArmState != ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                    extendArmState = ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (extendArmState == ExtendArmStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(eaInches1 - eaLimitLow) < 2) {
                        extendArmState = ExtendArmStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }
                // reset slides 0 pos
                if (extendArmState == ExtendArmStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        extendArm1.setPower(-0.1);
                        extendArm2.setPower(-0.1);
                    } else {
                        extendArm1.setPower(0);
                        extendArm2.setPower(0);
                        Motors.resetEncoders(extendArm1, extendArm2);
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, extendArm1, extendArm2);
                        extendArmState = ExtendArmStates.ZERO_POS_RESET;
                    }
                }
                // preset controls
                if (extendArmState == ExtendArmStates.MOVING_TO_PRESET) {
                    double pid = controller.calculate(eaInches1, slidesTARGET);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    // check if we are at the target by 50 encoders
                    if (Math.abs(eaInches1 - slidesTARGET) < 1) {
                        extendArmState = ExtendArmStates.PRESET_REACHED;
                    }
                }
                // preset
                if (gamepad1.b) {
                    extendArmState = ExtendArmStates.MOVING_TO_PRESET;
                }
                // telemetry for debugging
                telemetry.addData("extendArmState", extendArmState);
                telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
                telemetry.addData("target", slidesTARGET);
                telemetry.addData("eaCpos1", eaInches1);
                telemetry.addData("eaCpos2", eaInches2);
                telemetry.addData("preset error1", Math.abs(slidesTARGET - eaInches1));
                telemetry.addData("preset error2", Math.abs(slidesTARGET - eaInches2));
                telemetry.addData("preset errorAvg", (Math.abs(slidesTARGET - eaInches1) + Math.abs(slidesTARGET - eaInches2)) / 2);
                telemetry.addData("slides reset timer", resetTimer.milliseconds());
                telemetry.addData("extendArm1 Power", extendArm1.getPower());
                telemetry.addData("extendArm2 Power", extendArm2.getPower());
                telemetry.update();
            }
        }
    }
}
