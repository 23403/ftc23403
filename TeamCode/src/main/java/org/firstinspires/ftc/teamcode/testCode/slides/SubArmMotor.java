package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import xyz.nin1275.utils.Motors;

@Config("SubArmMotor config")
@TeleOp(name="Submersible motor test", group="test_ftc23403")
public class SubArmMotor extends LinearOpMode {
    public static int subArmCpos = 0;
    public static int SUB_ARM_MAX_POS = 1000;
    public static int SUB_ARM_MIN_POS = 0;
    public static double SUB_ARM_FF = 0.18;
    public static double SUB_ARM_SPEED = 0.8;
    public static double INIT_SPEED = -1;
    private final ElapsedTime resetTimer = new ElapsedTime();
    public static double TOLERANCE = 20;
    @Override
    public void runOpMode() {
        DcMotorEx subArm = hardwareMap.get(DcMotorEx.class, "subArm");
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
                // sliders code
                if (gamepad1.dpad_up) {
                    subArmCpos = SUB_ARM_MAX_POS;
                    subArm.setPower(SUB_ARM_SPEED);
                    subArm.setTargetPosition(subArmCpos);
                    subArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                } else if (gamepad1.dpad_down) {
                    subArmCpos = SUB_ARM_MIN_POS;
                    subArm.setPower(SUB_ARM_SPEED);
                    subArm.setTargetPosition(subArmCpos);
                    subArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                }
                // manual
                if (gamepad1.right_stick_y > 0.5) {
                    subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(SUB_ARM_SPEED + SUB_ARM_FF);
                    subArmCpos = subArm.getCurrentPosition();
                } else if (gamepad1.right_stick_y < 0.5) {
                    subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    subArm.setPower(-SUB_ARM_SPEED);
                    subArmCpos = subArm.getCurrentPosition();
                } else {
                    if (Math.abs(subArm.getCurrentPosition() - subArmCpos) < TOLERANCE) {
                        subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        subArm.setPower(0);
                    } else {
                        subArm.setPower(SUB_ARM_FF);
                        subArm.setTargetPosition(subArmCpos);
                        subArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    }
                }
                telemetry.addData("subArm Pos", subArm.getCurrentPosition());
                telemetry.addData("subArmCpos", subArmCpos);
                telemetry.addData("Power", subArm.getPower());
                telemetry.update();
            }
        }
    }
}
