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

    public static double CAMERA_VERTICAL_CENTER_OFFSET = -14.5;
    public static double CAMERA_HORIZONTAL_CENTER_OFFSET = -5;
    public static long MEMORY_TIMEOUT_MS = 500;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Servo rotation = hardwareMap.get(Servo.class, "rotation");
        Servo subArm = hardwareMap.get(Servo.class, "subArm1");

        subArm.scaleRange(0.45, 1);
        rotation.scaleRange(0.43, 0.55);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addLine("Ready. Press A to toggle alignment.");
        telemetry.update();

        waitForStart();

        double lastGoodTX = 0.0;
        double lastGoodTY = 0.0;
        double lastGoodTA = 0;
        long lastGoodTime = 0;

        boolean aligning = false;
        boolean aWasPressed = false;

        while (opModeIsActive()) {
            if (gamepad1.a && !aWasPressed) {
                aligning = !aligning;
                aWasPressed = true;
            } else if (!gamepad1.a) {
                aWasPressed = false;
            }

            LLResult result = limelight.getLatestResult();
            boolean validResult = (result != null && result.isValid());

            double txRaw = 0, tyRaw = 0, taRaw = 0;
            if (aligning) {
                if (validResult) {
                    txRaw = result.getTx();
                    tyRaw = result.getTy();
                    taRaw = result.getTa();
                    lastGoodTX = txRaw;
                    lastGoodTY = tyRaw;
                    lastGoodTA = taRaw;
                    lastGoodTime = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - lastGoodTime <= MEMORY_TIMEOUT_MS) {
                    txRaw = lastGoodTX;
                    tyRaw = lastGoodTY;
                    taRaw = lastGoodTA;
                } else {
                    rotation.setPosition(0.5);
                    subArm.setPosition(0.75);
                    telemetry.addLine("NO TARGET - RESETTING");
                    telemetry.update();
                    continue;
                }

                double distanceScale = 1.0 + (taRaw * 0.2);
                double tx = ((txRaw - CAMERA_HORIZONTAL_CENTER_OFFSET + 27) / 54.0) * distanceScale;
                double ty = (tyRaw - CAMERA_VERTICAL_CENTER_OFFSET + 20) / 40.0;

                rotation.setPosition(MathUtils.clamp(tx, 0.0, 1.0));
                subArm.setPosition(1.0 - MathUtils.clamp(ty, 0.0, 1.0));
            } else {
                rotation.setPosition(0.5);
                subArm.setPosition(0.75);
            }

            telemetry.addData("Aligning", aligning);
            telemetry.addData("Valid", validResult);
            telemetry.addData("tx", txRaw);
            telemetry.addData("ty", tyRaw);
            telemetry.addData("ta", taRaw);
            telemetry.addData("rotation pos", rotation.getPosition());
            telemetry.addData("subArm pos", subArm.getPosition());
            telemetry.update();
        }
    }
}
