package org.firstinspires.ftc.teamcode.testCode;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config("Limelight Nural Network Testing")
@TeleOp(name = "Limelight NN Test", group = "test_ftc23403")
public class LimelightNNTesting extends LinearOpMode {
    public static double CAMERA_VERTICAL_CENTER_OFFSET = 10;
    public static double CAMERA_HORIZONTAL_CENTER_OFFSET = 5;
    double lastGoodTX = 0.0;
    double lastGoodTY = 0.0;
    long lastGoodTime = 0;
    long MEMORY_TIMEOUT_MS = 500;
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Servo rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        Servo subArm = hardwareMap.get(Servo.class, "subArm1"); // 1x axon
        limelight.pipelineSwitch(1);
        limelight.start();
        subArm.scaleRange(0.45, 1);
        rotation.scaleRange(0.43, 0.55);
        telemetry.addLine("Initialized Limelight subsystem.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double txRaw;
            double tyRaw;
            if (result != null && result.isValid()) {
                txRaw = result.getTx();
                tyRaw = result.getTy();
                lastGoodTX = txRaw;
                lastGoodTY = tyRaw;
                lastGoodTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - lastGoodTime <= MEMORY_TIMEOUT_MS) {
                // Use last good result
                txRaw = lastGoodTX;
                tyRaw = lastGoodTY;
            } else {
                // if nothing found we go to limbo
                rotation.setPosition(0.5);
                subArm.setPosition(0.75);
                telemetry.addLine("NO TARGET - RESETTING");
                telemetry.update();
                continue;
            }
            // move servos
            double tx = (txRaw - CAMERA_HORIZONTAL_CENTER_OFFSET + 27) / 54.0;
            double ty = (tyRaw - CAMERA_VERTICAL_CENTER_OFFSET + 20) / 40.0;
            rotation.setPosition(MathUtils.clamp(tx, 0.0, 1.0));
            subArm.setPosition(1.0 - MathUtils.clamp(ty, 0.0, 1.0));
            // telemetry
            telemetry.addData("Valid", result != null && result.isValid());
            telemetry.addData("tx", txRaw);
            telemetry.addData("ty", tyRaw);
            telemetry.addData("tx norm", (txRaw - CAMERA_HORIZONTAL_CENTER_OFFSET + 27) / 54.0);
            telemetry.addData("ty norm", (tyRaw - CAMERA_VERTICAL_CENTER_OFFSET + 20) / 40.0);
            telemetry.addData("rotation pos", rotation.getPosition());
            telemetry.addData("subArm pos", subArm.getPosition());
            telemetry.addData("Using Last Good?", result == null && !result.isValid() && (System.currentTimeMillis() - lastGoodTime <= MEMORY_TIMEOUT_MS));
            telemetry.update();
        }
    }
}