package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config("Limelight CONFIG")
public class Limelight {
    private Telemetry telemetry;
    private final Limelight3A limelight;
    private final Servo rotationServo;
    private final Servo subArmServo;

    // Detection filtering
    public static double minArea = 0.01;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class ,"limelight");
        rotationServo = hardwareMap.get(Servo.class, "rotation");
        subArmServo = hardwareMap.get(Servo.class, "subArm");
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight.pipelineSwitch(0);
        // Set physical range limits
        rotationServo.scaleRange(0.43, 0.55);   // 0 = right, 1 = left
        subArmServo.scaleRange(0.45, 1.0);      // 0 = extended, 1 = retracted
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getTa() > minArea) {
            double tx = result.getTx();
            double ta = result.getTa();

            // Map tx [-27, 27] → [0, 1] for rotation servo
            double rotationPos = (tx + 27) / 54.0;
            rotationPos = Range.clip(rotationPos, 0.0, 1.0);
            rotationServo.setPosition(rotationPos);

            // Map ta [0, 100] → subArm [0 = extended, 1 = retracted]
            double subArmPos = 1.0 - (ta / 100.0);
            subArmPos = Range.clip(subArmPos, 0.0, 1.0);
            subArmServo.setPosition(subArmPos);

            telemetry.addData("isRunning", limelight.isRunning());
            telemetry.addData("latest result", limelight.getLatestResult());
            telemetry.addData("status", limelight.getStatus());
            telemetry.addData("isConnected", limelight.isConnected());
            telemetry.addData("Limelight Target", "Visible");
            telemetry.addData("tx", tx);
            telemetry.addData("ta", ta);
            telemetry.addData("Rotation Pos", rotationPos);
            telemetry.addData("SubArm Pos", subArmPos);
        } else {
            telemetry.addData("Limelight", "No target detected");
        }
        telemetry.update();
    }
}
