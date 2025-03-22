package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.testCode.PID_SquID.CustomPIDFTCoefficients;

@Config("PID Tune Slides")
@Autonomous(name="PID Tune Slides", group="test_ftc23403")
public class PIDTuneSlides extends OpMode {
    private DcMotorEx extendArm1;
    private DcMotorEx extendArm2;
    private PIDFController controller;
    public static CustomPIDFTCoefficients extendArmPID = new CustomPIDFTCoefficients(
            0,
            0,
            0,
            0,
            0);
    /**
     * Initialization code.
     */
    @Override
    public void init() {
        controller = new PIDFController(extendArmPID.P, extendArmPID.I, extendArmPID.D, extendArmPID.F);
        extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Use this to tune the extend arm.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the sensor distance in IN and the sensor RGBA values.
     */
    @Override
    public void loop() {
        controller.setPIDF(extendArmPID.P, extendArmPID.I, extendArmPID.D, extendArmPID.F);
        int eaCpos1 = extendArm1.getCurrentPosition();
        int eaCpos2 = extendArm2.getCurrentPosition();
        double power1 = controller.calculate(eaCpos1, extendArmPID.TARGET);
        double power2 = controller.calculate(eaCpos2, extendArmPID.TARGET);
        extendArm1.setPower(power1);
        extendArm2.setPower(power2);
        telemetry.addData("target", extendArmPID.TARGET);
        telemetry.addData("eaCpos1", eaCpos1);
        telemetry.addData("eaCpos2", eaCpos2);
        telemetry.addData("eaPower1", power1);
        telemetry.addData("eaPower2", power2);
        telemetry.addData("error1", Math.abs(extendArmPID.TARGET - eaCpos1));
        telemetry.addData("error2", Math.abs(extendArmPID.TARGET - eaCpos2));
        telemetry.addData("errorAvg", (Math.abs(extendArmPID.TARGET - eaCpos1) + Math.abs(extendArmPID.TARGET - eaCpos2)) / 2);
        telemetry.update();
    }
}
