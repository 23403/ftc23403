package org.firstinspires.ftc.teamcode.testCode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Turn Arm Corrections", group="test_ftc23403")
public class TurnArmCorrections extends LinearOpMode {


int  aPos = 0;
    @Override
    public void runOpMode() {
        DcMotor turnArm = hardwareMap.dcMotor.get("TurnArm");
        turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.left_bumper) {
                    turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turnArm.setPower(-0.5);
                    aPos = turnArm.getCurrentPosition();
                }
                else if (gamepad1.right_bumper) {
                    turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turnArm.setPower(0.5);
                    aPos = turnArm.getCurrentPosition();
                }
                else {
                    turnArm.setTargetPosition(aPos);
                    turnArm.setPower(1);
                    turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
        }
    }

}
