package org.firstinspires.ftc.teamcode.testCode;


import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.subsystems.LimelightState;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config("Limelight Nural Network Testing")
@TeleOp(name = "Limelight NN Test", group = "test_ftc23403")
public class LimelightNNTesting extends LinearOpMode {
    LimelightState llState = LimelightState.NO_SAMPLE_FOUND;
    @Override
    public void runOpMode() {
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        Servo rotation = hardwareMap.get(Servo.class, "rotation");
        Servo subArm = hardwareMap.get(Servo.class, "subArm1");
        subArm.scaleRange(0.45, 1);
        rotation.scaleRange(0.43, 0.55);
        subArm.setPosition(0.75);
        rotation.setPosition(0.5);
        Vision.Limelight limelight = new Vision.Limelight(limelight3A, llState);
        waitForStart();
        while (opModeIsActive()) {
            limelight.update();
            if (gamepad1.a) {
                subArm.setPosition(limelight.getSubmersible());
                rotation.setPosition(limelight.getRotation());
            } else if (gamepad1.b) {
                subArm.setPosition(0.75);
                rotation.setPosition(0.5);
            }
            telemetry.update();
        }
    }
}
