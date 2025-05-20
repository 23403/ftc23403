package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.SubArmSS;

import xyz.nin1275.controllers.PID;

@Config("PID Tune SubArm")
@Autonomous(name="PID Tune SubArm", group="test_ftc23403")
public class PIDTuneSubArm extends OpMode {
    private PID controller;
    public static double P = 0.6;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0.1;
    public static double TARGET = 0;
    public static double FULL = 5;
    public static double HALF = 2.5;
    public static double IN = 0;
    public static double CPR = 384.16; // counts per revolution
    public static double INCHES_PER_REV = 4.8; // how far the arm travels linearly per motor revolution
    private SubArmSS.PD subArm;


    /**
     * Initialization code.
     */
    @Override
    public void init() {
        // set the PID values
        controller = new PID(Math.sqrt(P), I, D);
        // hardware
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "SubArm");
        // reverse
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        // reset encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // breaks
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // init
        subArm = new SubArmSS.PD(
                motor,
                controller,
                F,
                CPR,
                INCHES_PER_REV
        );
        // combine both FTCDashboard and the regular telemetry
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        // telemetry
        telemetry.addLine("Use this to tune the subArm.");
        telemetry.update();
    }

    /**
     * This updates the FTC Dashboard telemetry with the ERROR values and target pos and current pos for easy tuning and debugging!
     */
    @Override
    public void loop() {
        // Update PID values
        controller.setPID(Math.sqrt(PIDTuneSlides.P), PIDTuneSlides.I, PIDTuneSlides.D);
        // presets
        if (gamepad1.a) subArm.moveTo(IN);
        if (gamepad1.b) subArm.moveTo(HALF);
        if (gamepad1.y) subArm.moveTo(FULL);
        if (gamepad1.x) subArm.moveTo(TARGET);
        // controls
        subArm.update(gamepad1.dpad_up, gamepad1.dpad_down);
        // telemetry for debugging
        telemetry.addLine("Tune PIDF:");
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.addData("currentState", subArm.getState());
        telemetry.addData("power", subArm.getPower());
        telemetry.addData("preset", subArm.getTarget());
        telemetry.addData("resetTimer S --> MS", subArm.getResetTimer().seconds() + " --> " + subArm.getResetTimer().milliseconds());
        telemetry.addData("pos", subArm.getInches());
        telemetry.update();
    }
}
