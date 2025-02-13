package org.firstinspires.ftc.teamcode.testCode;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config("Servos tester")
@TeleOp(name="Servo tester", group="test_ftc23403")
public class Servos extends LinearOpMode {

    public static double pos = 0;
    public static String servoName = "servo";
    public static boolean continues = false;

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (continues) {
                    CRServo servo = hardwareMap.get(CRServo.class, servoName);
                    servo.setPower(pos);
                    telemetry.addData("speed", pos);
                    telemetry.update();
                } else {
                    Servo servo = hardwareMap.get(Servo.class, servoName);
                    servo.setPosition(pos);
                    telemetry.addData("pos", servo.getPosition());
                    telemetry.update();
                }
            }
        }
    }

}
