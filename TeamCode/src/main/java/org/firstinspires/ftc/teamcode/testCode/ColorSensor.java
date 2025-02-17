package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.variables.old.VariablesOld;

@Autonomous(name="Color Sensor", group="test_ftc23403")
public class ColorSensor extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        waitForStart();
        telemetry.addData("Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
        telemetry.update();
    }
}
