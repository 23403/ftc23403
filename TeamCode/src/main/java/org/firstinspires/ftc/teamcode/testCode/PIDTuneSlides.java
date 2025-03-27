package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config("PID Tune Slides")
@Autonomous(name="PID Tune Slides", group="test_ftc23403")
public class PIDTuneSlides extends OpMode {
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private PIDController controller;
    public static double P = 0.000025;
    public static double I = 0;
    public static double D = 0.0005;
    public static double F = 0.1;
    public static double TARGET = 0;

    /**
     * Initialization code.
     */
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // reverse
        extendArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        // reset encoders
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // turn on the motors without the built in controller
        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // combine both FTCDashboard and the regular telemetry
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        // telemetry
        telemetry.addLine("Use this to tune the extend arm.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     */
    @Override
    public void loop() {
        // update the values so we can change them mid match
        controller.setPID(Math.sqrt(PIDTuneSlides.P), PIDTuneSlides.I, PIDTuneSlides.D);
        // grab current pos
        int eaCpos1 = extendArm1.getCurrentPosition();
        int eaCpos2 = extendArm2.getCurrentPosition();
        // calculate the power
        double pid = controller.calculate(eaCpos1, TARGET);
        // save the Force Feedback value
        double ff = PIDTuneSlides.F;
        // the final power value
        double power = pid + ff;
        // send the power to the motors
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        // telemetry for debugging
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
