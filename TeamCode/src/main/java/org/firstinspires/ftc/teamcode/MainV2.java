/***
 * MAIN V2
 * @author David Grieas - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v2
 * started recoding at 1/20/25  @  10:41 am
 * robot v2 finished building at 1/25/25
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.variables.VariablesNew;

@TeleOp(name="Main v2", group="ftc23403")
public class MainV2 extends LinearOpMode {
    /**
     * @TODO add arm code working
     * @TODO add intake and outtake code
     * @TODO have odometry working in teleOp
     * MAIN V2 BY DAVID
     * @author David Grieas - 23403 C{}de C<>nduct<>rs
     */
    @Override
    public void runOpMode() {
        // hardware
        VariablesNew variables = new VariablesNew();
        // motors
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        DcMotor turnArm = hardwareMap.dcMotor.get("TurnArm");
        DcMotor extendArm = hardwareMap.dcMotor.get("ExtendArm");
        // servos
        /*
        Servo servo = hardwareMap.get(Servo.class, "servo");
        CRServo crServo = hardwareMap.get(CRServo.class, "crServo");
        */
        // variables
        double wheelSpeed = variables.wheelSpeed;
        double extendArmSpeed = variables.extendArmSpeed;
        // Turn Arm Limits
        boolean taLimits = variables.taLimits;
        int taLimitHigh = variables.taLimitHigh;
        int taLimitLow = variables.taLimitLow;
        // Extend Arm Limits
        boolean eaLimits = variables.eaLimits;
        int eaLimitHigh = variables.eaLimitHigh;
        int eaLimitLow = variables.eaLimitLow;
        // starting POS
        boolean sp = variables.sp;
        int taSP = variables.taSP;
        int eaSP = variables.eaSP;
        // preset locations
        int specimenLoc = variables.specimenLoc;
        int submersalLoc = variables.submersalLoc;

        // Reverse one of the drive motors.
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // breaks
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // starting pos
        if (sp) {
            // turnArm
            turnArm.setTargetPosition(taSP);
            turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // extendArm
            extendArm.setTargetPosition(eaSP);
            extendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int taPOS = turnArm.getCurrentPosition();
                int eaPOS = extendArm.getCurrentPosition();
                // left wheels
                leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                // right wheels
                rightFrontDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                rightBackDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);

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
                // extendArm code
                if (eaLimits) {
                    if (eaPOS > eaLimitHigh && eaPOS < eaLimitLow) {
                        extendArm.setPower(gamepad1.right_stick_y);
                    } else if(eaPOS < eaLimitLow) {
                        if (gamepad1.right_bumper) {
                            extendArm.setPower(-extendArmSpeed);
                        } else {
                            extendArm.setPower(0);
                        }
                    } else if(eaPOS > eaLimitHigh) {
                        if (gamepad1.left_bumper) {
                            extendArm.setPower(extendArmSpeed);
                        } else {
                            extendArm.setPower(0);
                        }
                    } else {
                        extendArm.setPower(0);
                    }
                } else {
                    if (gamepad1.left_bumper) {
                        extendArm.setPower(-extendArmSpeed);
                    } else if (gamepad1.right_bumper){
                        extendArm.setPower(extendArmSpeed);
                    } else {
                        extendArm.setPower(0);
                    }
                }
                // preset code
                // specimen pos
                if (gamepad1.dpad_up) {
                    turnArm.setTargetPosition(specimenLoc);
                    turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turnArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                // submersal pos
                if (gamepad1.dpad_down) {
                    turnArm.setTargetPosition(submersalLoc);
                    turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turnArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                boolean manually;
                // odometry testing
                manually = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0 || gamepad1.right_stick_y > 0 || gamepad1.right_stick_y < 0;
                if (!manually) {
                    // odometry fixing itself
                }
                // telemetry
                telemetry.addData("LeftBackDrive", leftBackDrive.getCurrentPosition());
                telemetry.addData("LeftFrontDrive", leftFrontDrive.getCurrentPosition());
                telemetry.addData("RightBackDrive", rightBackDrive.getCurrentPosition());
                telemetry.addData("RightFrontDrive", rightFrontDrive.getCurrentPosition());
                telemetry.addData("Turn Arm Position:", taPOS);
                telemetry.addData("Extend Arm Position:", eaPOS);
                telemetry.addData("Manually moving robot?", manually);
                telemetry.update();
            }
        }
    }
}
