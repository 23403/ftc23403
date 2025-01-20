package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.variables.Variables;

@TeleOp(name="MainTest", group="ftc23403")
public class MainTest extends LinearOpMode {

    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor turnArm;
    private DcMotor extendArm;

    private int taSP;
    private int taSPM;
    private int taSPL;
    private boolean sp;
    private double wheelSpeed;
    private double extendArmSpeed;
    private int taLimitHigh;
    private int taLimitLow;
    private int taPL;
    private int taPLM;
    private int taPLL;
    private boolean pl;
    private boolean rtl;
    private boolean taLimits;

    /**
     * This OpMode offers POV (point-of-view) style TeleOp control for a direct drive robot.
     *
     * In this POV mode, the left joystick (up and down) moves the robot forward and back, and the
     * right joystick (left and right) spins the robot left (counterclockwise) and right (clockwise).
     */

    private int teMax = -2300; //upp
    private int teMin = -50; //down

    @Override
    public void runOpMode() {
        Variables variables = new Variables();
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        turnArm = hardwareMap.get(DcMotor.class, "TurnArm");
        extendArm = hardwareMap.get(DcMotor.class, "ExtendArm");

        //Turn Arm Limits
        taLimits = variables.taLimits;
        taLimitHigh = variables.taLimitHigh;
        taLimitLow = variables.taLimitLow;
        // starting POS
        taSP = variables.taSP;
        taSPM = variables.taSPM;
        taSPL = variables.taSPL;
        sp = variables.sp;
        wheelSpeed = variables.wheelSpeed;
        extendArmSpeed = variables.extendArmSpeed;
        // pickup low POS
        taPL = variables.taPL;
        taPLM = variables.taPLM;
        taPLL = variables.taPLL;
        pl = variables.pl;
        rtl = false;

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                int taPOS = turnArm.getCurrentPosition();
                // left wheels
                leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                // right wheels
                rightFrontDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                rightBackDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                // starting pos
                if (!sp) {
                    if (taPOS != taSP && taPOS != taSPM && taPOS != taSPL) {
                        if (taPOS < taSP) {
                            turnArm.setPower(0.7);
                        } else if (taPOS > taSP) {
                            turnArm.setPower(-0.7);
                        }
                    } else {
                        sp = true;
                    }
                }
                // turnArm code
                if (taLimits) {
                    if (taPOS > taLimitHigh && taPOS < taLimitLow) {
                        turnArm.setPower(gamepad1.right_stick_y);
                    } else if(taPOS < taLimitLow) {
                        if (gamepad1.right_stick_y > 0) {
                            turnArm.setPower(gamepad1.right_stick_y);
                        } else {
                            turnArm.setPower(0);
                        }
                    } else if(taPOS > taLimitHigh) {
                        if (gamepad1.right_stick_y < 0) {
                            turnArm.setPower(gamepad1.right_stick_y);
                        } else {
                            turnArm.setPower(0);
                        }
                    } else {
                        turnArm.setPower(0);
                    }
                } else {
                    if (gamepad1.right_stick_y < 0) {
                        turnArm.setPower(-gamepad1.right_stick_y);
                    } else if (gamepad1.right_stick_y > 0) {
                        turnArm.setPower(gamepad1.right_stick_y);
                    } else {
                        turnArm.setPower(0);
                    }
                }
                // extendArm no limits code
                if (gamepad1.left_bumper) {
                    extendArm.setPower(extendArmSpeed);
                } else if (gamepad1.right_bumper){
                    extendArm.setPower(extendArmSpeed);
                } else {
                    extendArm.setPower(0);
                }
                // telemetry
                telemetry.addData("LeftBackDrive", leftBackDrive.getPower());
                telemetry.addData("LeftFrontDrive", leftFrontDrive.getPower());
                telemetry.addData("RightBackDrive", rightBackDrive.getPower());
                telemetry.addData("RightFrontDrive", rightFrontDrive.getPower());
                telemetry.addData("Turn Arm Position:", taPOS);
                telemetry.update();
            }
        }
    }
}
