package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.utils.Motors;

@Config("SubArmMotorAAAAA config")
@TeleOp(name = "Submersible motorAAAAA test", group = "test_ftc23403")
public class SubArmMotorNEWWWW extends LinearOpMode {
    private int subArmCpos = 0;
    public static int slidesTARGET = 0;
    public static int SUB_ARM_MAX_POS = 1000;
    public static int SUB_ARM_MIN_POS = 0;
    public static double SUB_ARM_FF = 0.18;
    public static double SUB_ARM_SPEED = 0.8;
    public static double INIT_SPEED = -0.2;
    public static double TOLERANCE = 20;
    public static boolean saCorrection = true;

    private final ElapsedTime resetTimer = new ElapsedTime();
    private final ElapsedTime presetReachedTimer = new ElapsedTime();

    private SlidersStates sliderState = SlidersStates.FLOATING;
    private DcMotorEx subArm;
    private DcMotorEx.RunMode lastMode = null;

    private void setModeIfChanged(DcMotorEx.RunMode mode) {
        if (lastMode != mode) {
            subArm.setMode(mode);
            lastMode = mode;
        }
    }

    @Override
    public void runOpMode() {
        subArm = hardwareMap.get(DcMotorEx.class, "subArm");
        subArm.setDirection(DcMotorEx.Direction.REVERSE);

        // Pre-start zeroing
        resetTimer.reset();
        while (resetTimer.milliseconds() < 500) {
            subArm.setPower(INIT_SPEED);
        }
        subArm.setPower(0);
        Motors.resetEncoders(subArm);
        setModeIfChanged(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        resetTimer.reset();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double ff = saCorrection ? SUB_ARM_FF : 0;
                double stick = gamepad1.right_stick_y;

                // === Manual control ===
                if (stick > 0.5) {  // Move down
                    setModeIfChanged(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(SUB_ARM_SPEED + ff);
                    subArmCpos = subArm.getCurrentPosition();
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                } else if (stick < -0.5) {  // Move up
                    setModeIfChanged(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(-SUB_ARM_SPEED);
                    subArmCpos = subArm.getCurrentPosition();
                    sliderState = SlidersStates.MANUAL_MOVEMENT;
                }

                // === Presets ===
                if (gamepad1.dpad_up) {
                    slidesTARGET = SUB_ARM_MAX_POS;
                    sliderState = SlidersStates.MOVING_TO_PRESET;
                    presetReachedTimer.reset();
                } else if (gamepad1.dpad_down) {
                    slidesTARGET = SUB_ARM_MIN_POS;
                    sliderState = SlidersStates.MOVING_TO_PRESET;
                    presetReachedTimer.reset();
                }

                // === Hold Position or FF correction ===
                if (Math.abs(subArm.getCurrentPosition() - subArmCpos) < TOLERANCE && sliderState != SlidersStates.MOVING_TO_PRESET) {
                    setModeIfChanged(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(0);
                    sliderState = SlidersStates.FLOATING;
                } else if (sliderState != SlidersStates.MOVING_TO_PRESET &&
                        sliderState != SlidersStates.MANUAL_MOVEMENT &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS) {
                    subArm.setTargetPosition(subArmCpos);
                    setModeIfChanged(DcMotorEx.RunMode.RUN_TO_POSITION);
                    subArm.setPower(SUB_ARM_SPEED + ff);
                    sliderState = saCorrection ? SlidersStates.FORCE_FEED_BACK : SlidersStates.FLOATING;
                }

                // === Re-zeroing state logic ===
                if (Math.abs(subArm.getCurrentPosition() - SUB_ARM_MIN_POS) < TOLERANCE &&
                        sliderState != SlidersStates.RESETTING_ZERO_POS &&
                        sliderState != SlidersStates.ZERO_POS_RESET) {
                    sliderState = SlidersStates.WAITING_FOR_RESET_CONFIRMATION;
                    resetTimer.reset();
                }

                if (sliderState == SlidersStates.WAITING_FOR_RESET_CONFIRMATION) {
                    if (resetTimer.milliseconds() > 200 &&
                            Math.abs(subArm.getCurrentPosition() - SUB_ARM_MIN_POS) < TOLERANCE) {
                        sliderState = SlidersStates.RESETTING_ZERO_POS;
                        resetTimer.reset();
                    }
                }

                if (sliderState == SlidersStates.RESETTING_ZERO_POS) {
                    if (resetTimer.milliseconds() < 200) {
                        subArm.setPower(-0.1);
                    } else {
                        subArm.setPower(0);
                        Motors.resetEncoders(subArm);
                        setModeIfChanged(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        subArmCpos = 0;
                        sliderState = SlidersStates.ZERO_POS_RESET;
                    }
                }

                // === Preset movement control ===
                if (sliderState == SlidersStates.MOVING_TO_PRESET) {
                    subArm.setTargetPosition(slidesTARGET);
                    setModeIfChanged(DcMotorEx.RunMode.RUN_TO_POSITION);
                    subArm.setPower(SUB_ARM_SPEED + ff);

                    if (Math.abs(subArm.getCurrentPosition() - slidesTARGET) < TOLERANCE) {
                        if (presetReachedTimer.milliseconds() > 200) {
                            subArmCpos = slidesTARGET;
                            sliderState = SlidersStates.PRESET_REACHED;
                        }
                    }
                }

                // === Telemetry ===
                telemetry.addData("State", sliderState);
                telemetry.addData("subArm Pos", subArm.getCurrentPosition());
                telemetry.addData("Target", slidesTARGET);
                telemetry.addData("subArmCpos", subArmCpos);
                telemetry.addData("Power", subArm.getPower());
                telemetry.update();
            }
        }
    }
}
