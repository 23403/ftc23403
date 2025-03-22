package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.testCode.PID_SquID.CustomPIDFTCoefficients;

@Config("PID Tune Slides")
@Autonomous(name="PID Tune Slides", group="test_ftc23403")
public class PIDTuneSlides extends OpMode {
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private PIDController controller;
    public static double P = 0.01;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.1;
    public static double TARGET = 0;
    /**
     * Initialization code.
     */
    @Override
    public void init() {
        controller = new PIDController(P, I, D);
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        extendArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Use this to tune the extend arm.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the sensor distance in IN and the sensor RGBA values.
     */
    @Override
    public void loop() {
        controller.setPID(P, I, D);
        int eaCpos1 = extendArm1.getCurrentPosition();
        int eaCpos2 = extendArm2.getCurrentPosition();
        double pid = controller.calculate(eaCpos1, TARGET);
       //  double pid2 = controller.calculate(eaCpos2, TARGET);
//        double ff = Math.cos(Math.toRadians(TARGET/tickPerRevolution)) * F;
        double ff = F;
        double power = pid + ff;
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        telemetry.addData("PIDF", "P: " + P + " I: " + I + " D: " + D + " F: " + F);
        telemetry.addData("target", TARGET);
        telemetry.addData("eaCpos1", eaCpos1);
        telemetry.addData("eaCpos2", eaCpos2);
        telemetry.addData("eaPower", power);
        telemetry.addData("error1", Math.abs(TARGET - eaCpos1));
        telemetry.addData("error2", Math.abs(TARGET - eaCpos2));
        telemetry.addData("errorAvg", (Math.abs(TARGET - eaCpos1) + Math.abs(TARGET - eaCpos2)) / 2);
        telemetry.update();
    }
}
