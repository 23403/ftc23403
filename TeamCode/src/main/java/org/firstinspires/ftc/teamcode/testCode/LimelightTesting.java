package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightPy;

@Config("Limelight Testing")
@TeleOp(name = "Limelight Test", group = "test_ftc23403")
public class LimelightTesting extends LinearOpMode {
    public static boolean updateLimelight = true;
    public static boolean limelightTelemetry = false;
    @Override
    public void runOpMode() {
        Limelight3A cam = hardwareMap.get(Limelight3A.class, "limelight");
        Servo rotation = hardwareMap.get(Servo.class, "rotation");
        Servo subArm1 = hardwareMap.get(Servo.class, "subArm1");

        LimelightPy limelight = new LimelightPy(cam, rotation, subArm1, telemetry);

        cam.start();
        telemetry.addLine("Initialized Limelight subsystem.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!limelightTelemetry) limelight.updateTelemetry(telemetry);
            if (updateLimelight) limelight.updateLimelight(cam);
            limelight.update();
            if (limelightTelemetry) {
                telemetry.addData("isRunning", cam.isRunning());
                telemetry.addData("latest result", cam.getLatestResult());
                telemetry.addData("status", cam.getStatus());
                telemetry.addData("isConnected", cam.isConnected());
            }
            telemetry.update();
        }
    }
}