package org.firstinspires.ftc.teamcode.testCode.slides;

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

import org.firstinspires.ftc.teamcode.variables.enums.extendArmStates;

import xyz.nin1275.utils.Timer;

@Config("PID Tuned Slides Test")
@TeleOp(name="PID Tuned Slides Test", group="test_ftc23403")
public class PIDTunedSlidesTest extends LinearOpMode {
    public static boolean eaCorrection = true;
    public static double presetPos = 5;
    public static double eaLimitHigh = Double.MAX_VALUE;
    public static double eaLimitLow = Double.MIN_VALUE;
    private static extendArmStates extendArmState = extendArmStates.FLOATING;

    @Override
    public void runOpMode() throws InterruptedException {
        PIDController controller = new PIDController(Math.sqrt(P), I, D);
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // telemetry.addData("currentState", extendArmSS.getCurrentState());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Update PID values
                controller.setPID(Math.sqrt(P), I, D);
                // Get current positions
                int eaTicks1 = extendArm1.getCurrentPosition();
                int eaTicks2 = extendArm2.getCurrentPosition();
                // Convert ticks to inches
                // counts per revolution
                double CPR = 384.16;
                // vars
                double ff = eaCorrection ? F : 0;
                // how far the arm travels linearly per motor revolution
                double INCHES_PER_REV = 4.8;
                double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
                double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;


                // controls
                if (gamepad1.dpad_up && eaInches1 < eaLimitHigh) {
                    double pid = controller.calculate(eaInches1, eaLimitHigh);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = extendArmStates.MANUAL_MOVEMENT;
                } else if (gamepad1.dpad_down && eaInches1 > eaLimitLow) {
                    double pid = controller.calculate(eaInches1, eaLimitLow);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    extendArmState = extendArmStates.MANUAL_MOVEMENT;
                } else if (Math.abs(eaInches1 - eaLimitLow) > 2 && extendArmState != extendArmStates.MOVING_TO_PRESET) {
                    extendArm1.setPower(ff);
                    extendArm2.setPower(ff);
                    if (extendArmState == extendArmStates.PRESET_REACHED) Timer.wait(500);
                    extendArmState = eaCorrection ? extendArmStates.FORCE_FEED_BACK : extendArmStates.FLOATING;
                }
                if (gamepad1.b) {
                    extendArmState = extendArmStates.MOVING_TO_PRESET;
                }
                // states
                if (Math.abs(eaInches1 - eaLimitHigh) < 1) {
                    extendArmState = extendArmStates.MAX_POS;
                } else if (Math.abs(eaInches1 - eaLimitLow) < 1) {
                    extendArmState = extendArmStates.LOW_POS;
                }
                // preset controls
                if (extendArmState == extendArmStates.MOVING_TO_PRESET) {
                    double pid = controller.calculate(eaInches1, presetPos);
                    double rawPower = pid + ff;
                    double syncError = eaInches1 - eaInches2;
                    double correction = syncError * K;
                    extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
                    extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
                    // check if we are at the target by 50 encoders
                    if (Math.abs(eaInches1 - presetPos) < 1) {
                        extendArmState = extendArmStates.PRESET_REACHED;
                    }
                }



                // telemetry for debugging
                telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
                telemetry.addData("target", presetPos);
                telemetry.addData("eaCpos1", eaInches1);
                telemetry.addData("eaCpos2", eaInches2);
                telemetry.update();
            }
        }
    }
}
