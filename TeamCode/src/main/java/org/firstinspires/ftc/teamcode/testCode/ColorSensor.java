package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Color Sensor", group="test_ftc23403")
public class ColorSensor extends OpMode {
    private Telemetry telemetryA;
    private ColorRangeSensor sensor;

    /**
     * Initialization code.
     */
    @Override
    public void init() {
        sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Use this to calibrate the color sensor values to detect the blocks.");
        telemetryA.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the sensor distance in IN and the sensor RGBA values.
     */
    @Override
    public void loop() {
        if (gamepad1.x) {
            telemetry.addData("Pick Up Yellow Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
            telemetryA.addData("Pick Up Yellow Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        }
        if (gamepad1.y) {
            telemetry.addData("Pick Up Blue Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
            telemetryA.addData("Pick Up Blue Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        }
        if (gamepad1.b) {
            telemetry.addData("Pick Up Red Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
            telemetryA.addData("Pick Up Red Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        }
        if (gamepad1.a) {
            telemetry.addData("Pick Up Distance", sensor.getDistance(DistanceUnit.MM));
            telemetryA.addData("Pick Up Distance", sensor.getDistance(DistanceUnit.MM));
        }
        if (gamepad1.dpad_up) {
            telemetry.addData("Grabbed Yellow Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
            telemetryA.addData("Grabbed Yellow Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        }
        if (gamepad1.dpad_right) {
            telemetry.addData("Grabbed Blue Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
            telemetryA.addData("Grabbed Blue Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        }
        if (gamepad1.dpad_down) {
            telemetry.addData("Grabbed Red Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
            telemetryA.addData("Grabbed Red Sample", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        }
        if (gamepad1.dpad_left) {
            telemetry.addData("Grabbed Distance", sensor.getDistance(DistanceUnit.MM));
            telemetryA.addData("Grabbed Distance", sensor.getDistance(DistanceUnit.MM));
        }

        telemetryA.addLine("/**\n" +
                " *   X / ▢         - Save Pick Up Yellow Sample\n" +
                " *   Y / Δ         - Save Pick Up Blue Sample\n" +
                " *   B / O         - Save Pick Up Red Sample\n" +
                " *   A / X         - Save Pick Up Distance\n" +
                " * \n" +
                " *   ↑ / ↑         - Save Grabbed Yellow Sample\n" +
                " *   → / →         - Save Grabbed Blue Sample\n" +
                " *   ↓ / ↓         - Save Grabbed Red Sample\n" +
                " *   ← / ←         - Save Grabbed Distance\n" +
                " *                                    \n" +
                " *                                    \n" +
                " *                    ________        \n" +
                " *                   / ______ \\\n" +
                " *     ------------.-'   _  '-..+             \n" +
                " *              /   _  ( Y )  _  \\                  \n" +
                " *             |  ( X )  _  ( B ) |     \n" +
                " *        ___  '.      ( A )     /|      \n" +
                " *      .'    '.    '-._____.-'  .'       \n" +
                " *     |       |                 |                      \n" +
                " *      '.___.' '.               |          \n" +
                " *               '.             /             \n" +
                " *                \\.          .'              \n" +
                " *                  \\________/\n" +
                " */");
        telemetryA.addData("Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
        telemetryA.addData("Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        telemetryA.update();
        telemetry.update();
    }
}
