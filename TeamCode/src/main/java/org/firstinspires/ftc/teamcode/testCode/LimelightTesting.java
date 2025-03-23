package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@Config("Limelight Testing")
@Autonomous(name="Limelight Testing", group="test_ftc23403")
public class LimelightTesting extends OpMode {
    private Servo wristServo;
    public static double kRotationFactor = 0.5 / 27.0;
    /**
     * Initialization code.
     */
    @Override
    public void init() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        wristServo = hardwareMap.get(Servo.class, "wrist2");
        // combine both FTCDashboard and the regular telemetry
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        // telemetry
        telemetry.addLine("Use this to config limelight.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     */
    @Override
    public void loop() {
        try {
            // Get Limelight data from HTTP
            JSONObject limelightData = getLimelightData("http://10.0.0.11:5801/limelight-data");
            double tx = limelightData.getDouble("tx");

            // Convert tx to servo position (scale from -27 to 27 degrees into 0-1 range)
            double servoPosition = 0.5 + (tx * kRotationFactor);

            // Clamp servo position to valid range
            servoPosition = Math.max(0, Math.min(1, servoPosition));

            // Move the wrist servo
            wristServo.setPosition(servoPosition);

            // Telemetry for debugging
            telemetry.addData("tx", tx);
            telemetry.addData("Servo Position", servoPosition);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }

    // Function to get Limelight data via HTTP request
    private JSONObject getLimelightData(String urlString) throws Exception {
        URL url = new URL(urlString);
        HttpURLConnection conn = (HttpURLConnection) url.openConnection();
        conn.setRequestMethod("GET");
        BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
        String inputLine;
        StringBuilder response = new StringBuilder();
        while ((inputLine = in.readLine()) != null) {
            response.append(inputLine);
        }
        in.close();
        return new JSONObject(response.toString());
    }
}
