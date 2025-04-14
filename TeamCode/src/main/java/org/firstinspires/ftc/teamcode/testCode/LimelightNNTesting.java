package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightPy;

@Config("Limelight Nural Network Testing")
@TeleOp(name = "Limelight NN Test", group = "test_ftc23403")
public class LimelightNNTesting extends LinearOpMode {
    public static boolean updateLimelight = true;
    public static boolean limelightTelemetry = false;
    @Override
    public void runOpMode() {
        Limelight3A cam = hardwareMap.get(Limelight3A.class, "limelight");

        Limelight limelight = new Limelight(hardwareMap, telemetry);

        cam.start();
        telemetry.addLine("Initialized Limelight subsystem.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            limelight.update();
        }
    }
}