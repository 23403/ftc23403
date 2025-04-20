package org.firstinspires.ftc.teamcode.testCode;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Turn Arm Limits", group="test_ftc23403")
public class TurnArmLimits extends LinearOpMode {

    private int MAX_TURN = 7 ;
    private int MIN_TURN = -50;

    @Override
    public void runOpMode() {
        DcMotor turnArm = hardwareMap.dcMotor.get("TurnArm");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int turnPosition = turnArm.getCurrentPosition();
                if (gamepad1.left_bumper && turnPosition > MAX_TURN) {
                    turnArm.setPower(-0.5);
                } else if (gamepad1.right_bumper && turnPosition > MIN_TURN) {
                    turnArm.setPower(0.5);
                } else {
                    turnArm.setPower(0);
                }
                telemetry.addData("Turn_pos", turnPosition);
                telemetry.update();
            }
        }
    }

}
