package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config("F Drift Logger")
@Autonomous(name = "F Drift Logger", group = "test_ftc23403")
public class FeedforwardDriftLogger extends OpMode {

    private DcMotorEx subArm;

    // Tuning parameters
    public static double POWER_TO_EXTEND = 0.6;
    public static double HOLD_TIME = 3.0;
    public static double START_F = 0.00;
    public static double F_INCREMENT = 0.01;
    public static double STEP_TIME = 2.0;
    public static double MAX_F = 0.2;

    // Internal tracking
    private double currentF = START_F;
    private double startTime;
    private boolean extended = false;

    private int lastEncoder = 0;
    private double lastStepTime = 0.0;

    @Override
    public void init() {
        subArm = hardwareMap.get(DcMotorEx.class, "SubArm");
        subArm.setDirection(DcMotorSimple.Direction.REVERSE);
        subArm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        subArm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to test feedforward with drift logging.");
        telemetry.update();
    }

    @Override
    public void start() {
        startTime = getRuntime();
        lastStepTime = startTime;
        lastEncoder = subArm.getCurrentPosition();
    }

    @Override
    public void loop() {
        double elapsed = getRuntime() - startTime;

        if (!extended && elapsed < HOLD_TIME) {
            subArm.setPower(POWER_TO_EXTEND);
            telemetry.addLine("Extending to full...");
        } else {
            extended = true;

            double now = getRuntime();
            int currentEncoder = subArm.getCurrentPosition();

            // Compute which step we're in
            int stepIndex = (int) ((now - startTime - HOLD_TIME) / STEP_TIME);
            currentF = START_F + stepIndex * F_INCREMENT;
            currentF = Math.min(currentF, MAX_F);

            subArm.setPower(currentF);

            // Only calculate drift once per step
            if ((now - lastStepTime) >= STEP_TIME) {
                int deltaTicks = currentEncoder - lastEncoder;
                double deltaTime = now - lastStepTime;
                double driftRate = deltaTicks / deltaTime;

                telemetry.addLine("F Step Complete:");
                telemetry.addData("  F value", currentF);
                telemetry.addData("  Drift (ticks/sec)", driftRate);
                telemetry.addData("  Encoder delta", deltaTicks);
                telemetry.addData("  Elapsed in step", deltaTime);

                // Update last tracking values
                lastStepTime = now;
                lastEncoder = currentEncoder;
            }

            telemetry.addData("Current Encoder", currentEncoder);
            telemetry.addData("Current F", currentF);
        }

        telemetry.update();
    }
}
