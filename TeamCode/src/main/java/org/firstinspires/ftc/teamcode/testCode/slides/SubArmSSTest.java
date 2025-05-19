package org.firstinspires.ftc.teamcode.testCode.slides;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.SubArmSS;

import xyz.nin1275.controllers.PID;

@Config("SubArmSS Test")
@TeleOp(name="SubArmSS Test", group="test_ftc23403")
public class SubArmSSTest extends LinearOpMode {
    // vars
    public static boolean saCorrection = true;
    public static double presetPos = 5;
    public static double CPR = 384.16; // counts per revolution
    public static double INCHES_PER_REV = 4.8; // how far the arm travels linearly per motor revolution
    public static double P = 0.6;
    public static double F = 0.1;
    public static double saLimitHigh = 1000;
    public static double saLimitLow = -1000;

    @Override
    public void runOpMode() {
        // hardware
        DcMotorEx subArm = hardwareMap.get(DcMotorEx.class, "SubArm");
        subArm.setDirection(DcMotor.Direction.REVERSE);
        PID controller = new PID(Math.sqrt(P), 0, 0);
        SubArmSS subArmSS = new SubArmSS(subArm, controller, F, CPR, INCHES_PER_REV);
        // telemetry
        telemetry.addData("currentState", subArmSS.getState());
        telemetry.addData("power", subArmSS.getPower());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // values
                subArmSS.setSaCorrection(saCorrection);
                subArmSS.setLimits(saLimitHigh, saLimitLow);
                subArmSS.updatePIDKFValues(P, 0, 0, 0, F);
                // preset
                if (gamepad1.b) {
                    subArmSS.moveTo(presetPos);
                }
                // update
                subArmSS.update(gamepad1.dpad_up, gamepad1.dpad_down);
                // telemetry
                telemetry.addData("currentState", subArmSS.getState());
                telemetry.addData("power", subArmSS.getPower());
                telemetry.addData("preset", subArmSS.getTarget());
                telemetry.addData("resetTimer S --> MS", subArmSS.getResetTimer().seconds() + " --> " + subArmSS.getResetTimer().milliseconds());
                telemetry.addData("pos", subArmSS.getInches1());
                telemetry.update();
            }
        }
    }
}
