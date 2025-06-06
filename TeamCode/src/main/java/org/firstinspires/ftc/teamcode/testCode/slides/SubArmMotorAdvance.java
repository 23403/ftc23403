package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Timer;

@Config("SubArmMotorA config")
@TeleOp(name="Submersible motorA test", group="test_ftc23403")
public class SubArmMotorAdvance extends LinearOpMode {
    private int subArmCpos = 0;
    public static int slidesTARGET = 0;
    public static int SUB_ARM_MAX_POS = 1000;
    public static int SUB_ARM_MIN_POS = 0;
    public static double SUB_ARM_FF = 0.18;
    public static double SUB_ARM_SPEED = 0.8;
    public static double INIT_SPEED = -1;
    private final ElapsedTime resetTimer = new ElapsedTime();
    private SlidersStates sliderState = SlidersStates.FLOATING;
    public static boolean saCorrection = true;
    public static double TOLERANCE = 20;
    @Override
    public void runOpMode() {
        DcMotorEx subArm = hardwareMap.get(DcMotorEx.class, "subArm");
        subArm.setDirection(DcMotorEx.Direction.REVERSE);
        // init stuff
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            subArm.setPower(INIT_SPEED);
        }
        subArm.setPower(0);
        Motors.resetEncoders(subArm);
        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm);
        resetTimer.reset();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double ff = saCorrection ? SUB_ARM_FF : 0;
                // sliders code
                if (gamepad1.dpad_up) {
                    slidesTARGET = SUB_ARM_MAX_POS;
                    sliderState = SlidersStates.MOVING_TO_PRESET;
                } else if (gamepad1.dpad_down) {
                    slidesTARGET = SUB_ARM_MIN_POS;
                    sliderState = SlidersStates.MOVING_TO_PRESET;
                }
                // manual
                if (gamepad1.right_stick_y > 0.5) {
                    subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(SUB_ARM_SPEED + ff);
                    subArmCpos = subArm.getCurrentPosition();
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else if (gamepad1.right_stick_y < 0.5) {
                    subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(-SUB_ARM_SPEED);
                    subArmCpos = subArm.getCurrentPosition();
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else {
                    if (Math.abs(subArm.getCurrentPosition() - subArmCpos) < TOLERANCE) {
                        subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        subArm.setPower(0);
                        sliderState = SlidersStates.FLOATING;
                    } else if (sliderState != SlidersStates.MOVING_TO_PRESET) {
                        subArm.setPower(ff);
                        subArm.setTargetPosition(subArmCpos);
                        subArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        if (sliderState == SlidersStates.PRESET_REACHED) Timer.wait(500);
                        sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
                    }
                }

                // states
                if (Math.abs(subArm.getCurrentPosition() - SUB_ARM_MAX_POS) < TOLERANCE && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    sliderState = SlidersStates.MAX_POS;
                } else if (Math.abs(subArm.getCurrentPosition() - SUB_ARM_MIN_POS) < TOLERANCE &&
                        sliderState != SlidersStates.MOVING_TO_PRESET &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS &&
                        sliderState != SlidersStates.ZERO_POS_RESET &&
                        sliderState != SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }
                // pre resetting slides pos
                if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 && Math.abs(subArm.getCurrentPosition() - SUB_ARM_MIN_POS) < TOLERANCE) {
                        sliderState = SlidersStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }
                // reset slides 0 pos
                if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        subArm.setPower(-0.1);
                    } else {
                        subArm.setPower(0);
                        Motors.resetEncoders(subArm);
                        Motors.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, subArm);
                        sliderState = SlidersStates.ZERO_POS_RESET;
                    }
                }
                // preset controls
                if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                    subArm.setPower(SUB_ARM_SPEED);
                    subArm.setTargetPosition(slidesTARGET);
                    subArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    if (Math.abs(subArm.getCurrentPosition() - slidesTARGET) < TOLERANCE) sliderState = SlidersStates.PRESET_REACHED;
                }
            }
        }
    }
}
