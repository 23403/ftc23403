package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.TelemetryM;

@Autonomous(name="Analog Test", group="test_ftc23403")
public class AnalogTest extends OpMode {
    private TelemetryM telemetryM;
    private AnalogInput subArm1;
    private AnalogInput arm1;
    private AnalogInput arm2;
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    /**
     * Initialization code.
     */
    @Override
    public void init() {
        subArm1 = hardwareMap.get(AnalogInput.class, "subArm1");
        arm1 = hardwareMap.get(AnalogInput.class, "arm1");
        arm2 = hardwareMap.get(AnalogInput.class, "arm2");
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryM = new TelemetryM(telemetry, true);
        // reverse
        extendArm1.setDirection(DcMotorSimple.Direction.REVERSE);
        // reset encoders
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on the motors without the built in controller
        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // telemetry
        telemetryM.addLine("Use this to save the analog positions.");
        telemetryM.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the sensor distance in IN and the sensor RGBA values.
     */
    @Override
    public void loop() {
        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        double subArmPos1 = subArm1.getVoltage() / 3.3 * 360;
        double armPos1 = arm1.getVoltage() / 3.3 * 360;
        double armPos2 = arm2.getVoltage() / 3.3 * 360;
        // odo
        double eaCpos1 = extendArm1.getCurrentPosition();
        double eaCpos2 = extendArm2.getCurrentPosition();
        if (gamepad1.x) {
            telemetryM.addData("Save Sub Arm DeExtended", subArmPos1);
        }
        if (gamepad1.y) {
            telemetryM.addData("Save Arm Forward", armPos1 + " - " + armPos2);
        }
        if (gamepad1.b) {
            telemetryM.addData("Save Sub Arm Extended", subArmPos1);
        }
        if (gamepad1.a) {
            telemetryM.addData("Save Arm Back", armPos1 + " - " + armPos2);
        }
        if (gamepad1.dpad_up) {
            telemetryM.addData("Save ExtendArm Max", eaCpos1 + " - " + eaCpos2);
        }
        if (gamepad1.dpad_down) {
            telemetryM.addData("Save ExtendArm Min", eaCpos1 + " - " + eaCpos2);
        }
        // telemetry
        telemetryM.addLine("/**\n" +
                " *   X / ▢         - Save Arm Back\n" +
                " *   Y / Δ         - Save Arm Forward\n" +
                " *   B / O         - Save Sub Arm Extended\n" +
                " *   A / X         - Save Sub Arm DeExtended\n" +
                " * \n" +
                " *   ↑ / ↑         - Save ExtendArm Max\n" +
                " *   → / →         - N/A\n" +
                " *   ↓ / ↓         - Save ExtendArm Min\n" +
                " *   ← / ←         - N/A\n" +
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
        telemetryM.addData("subArm1 Position:", subArmPos1);
        telemetryM.addData("arm1 Position:", armPos1);
        telemetryM.addData("arm2 Position:", armPos2);
        telemetryM.addData("eaCpos1 Position:", eaCpos1);
        telemetryM.addData("eaCpos2 Position:", eaCpos2);
        telemetryM.update();
    }
}
