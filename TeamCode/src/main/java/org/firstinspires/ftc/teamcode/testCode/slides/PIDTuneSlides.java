package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Config("PID Tune Slides")
@Autonomous(name="PID Tune Slides", group="test_ftc23403")
public class PIDTuneSlides extends OpMode {
    private CachingDcMotorEx extendArm1;
    private CachingDcMotorEx extendArm2;
    private PIDController controller;
    public static double P = 0.05;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.35;
    public static double K = 0.05;
    public static double TARGET = 0;
    public static double CPR = 145.1; // counts per revolution
    public static double INCHES_PER_REV = 4.2; // how far the arm travels linearly per motor revolution


    /**
     * Initialization code.
     */
    @Override
    public void init() {
        // set the PID values
        controller = new PIDController(Math.sqrt(P), I, D);
        // hardware
        extendArm1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm1"));
        extendArm2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm2"));
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
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneSlides.P), PIDTuneSlides.I, PIDTuneSlides.D);
        // Get current positions
        int eaTicks1 = extendArm1.getCurrentPosition();
        int eaTicks2 = extendArm2.getCurrentPosition();
        // Convert ticks to inches
        double eaInches1 = (eaTicks1 / CPR) * INCHES_PER_REV;
        double eaInches2 = (eaTicks2 / CPR) * INCHES_PER_REV;
        // Calculate PID only on one motor (leader)
        double pid = controller.calculate(eaInches1, TARGET);
        double ff = F;
        double rawPower = pid + ff;
        // Calculate sync error
        double syncError = eaInches1 - eaInches2;
        // Calculate correction power
        double correction = syncError * K;
        // Apply power
        extendArm1.setPower(Math.max(-1, Math.min(1, rawPower))); // leader
        extendArm2.setPower(Math.max(-1, Math.min(1, (rawPower + correction)))); // follower with correction
        // telemetry for debugging
        telemetry.addData("PIDFK", "P: " + P + " I: " + I + " D: " + D + " F: " + F + " K: " + K);
        telemetry.addData("target", TARGET);
        telemetry.addData("eaCpos1", eaInches1);
        telemetry.addData("eaCpos2", eaInches2);
        telemetry.addData("eaPowerRAW", rawPower);
        telemetry.addData("eaPower", Math.max(-1, Math.min(1, rawPower)));
        telemetry.addData("eaPower SYNC ERROR RAW", rawPower + correction);
        telemetry.addData("error1", Math.abs(TARGET - eaInches1));
        telemetry.addData("error2", Math.abs(TARGET - eaInches2));
        telemetry.addData("errorAvg", (Math.abs(TARGET - eaInches1) + Math.abs(TARGET - eaInches2)) / 2);
        telemetry.addData("errorSync", syncError);
        telemetry.update();
    }
}
