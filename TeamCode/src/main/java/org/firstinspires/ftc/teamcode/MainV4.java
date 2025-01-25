package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.security.PrivateKey;


@TeleOp(name="Main v4", group="ftc23403")
public class MainV4 extends LinearOpMode {

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
