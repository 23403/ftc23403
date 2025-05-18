package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;

@Autonomous(name="Current Tracker", group="tools_ftc23403")
public class CurrentTracker extends OpMode {
    private TelemetryM telemetryM;
    /**
     * Initialization code.
     */
    @Override
    public void init() {
        // hardware
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = new TelemetryM(telemetry, true);
        // init
        LynxUtils.initLynx(hardwareMap);
        // telemetry
        telemetryM.addLine("Use this to save the analog positions.");
        telemetryM.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the sensor distance in IN and the sensor RGBA values.
     */
    @Override
    public void loop() {
        // colors
        LynxUtils.setLynxColor((int)(Math.random()*255), (int)(Math.random()*255), (int)(Math.random()*255));
        // telemetry
        telemetryM.addData("Control Hub Current", LynxUtils.getControlHubCurrent());
        telemetryM.addData("Expansion Hub Current", LynxUtils.getExpansionHubCurrent());
        telemetryM.addData("Servo Hub Current", LynxUtils.getServoHubCurrent());
        telemetryM.update();
    }

    @Override
    public void stop() {
        // stop
        LynxUtils.setLynxColor(0, 255, 0);
    }
}
