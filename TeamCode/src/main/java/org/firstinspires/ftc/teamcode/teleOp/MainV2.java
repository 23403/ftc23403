/***
 * MAIN V2
 * @author David Grieas - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v2
 * started recoding at 1/20/25  @  10:41 am
 * robot v2 finished building at 1/30/25
 */
package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.variables.VariablesNew;

@TeleOp(name="Main v2", group="ftc23403")
public class MainV2 extends LinearOpMode {
    /**
     * @TODO add proper WORKING limits to the turnArm and extendArms
     * @TODO add intake and outtake code
     * @TODO add wrist and arm code
     * @TODO add the preset locations positions
     * @TODO have odometry working in teleOp
     * @TODO make the slides correction more smooth
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
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        // servos
        /*
        Servo servo = hardwareMap.get(Servo.class, "servo");
        CRServo crServo = hardwareMap.get(CRServo.class, "crServo");
        */
        // variables
        double wheelSpeed = variables.wheelSpeed;
        double extendArmSpeed = variables.extendArmSpeed;
        double turnArmSpeedM = variables.turnArmSpeed;
        double turnArmSpeed = (gamepad1.right_stick_y > turnArmSpeedM) ? turnArmSpeedM : Math.abs(gamepad1.left_stick_y); // will ALWAYS return POSITIVE value!
        // Turn Arm
        boolean taLimits = variables.taLimits;
        boolean taCorrection = variables.taCorrection;
        int taLimitHigh = variables.taLimitHigh;
        int taLimitLow = variables.taLimitLow;
        // Extend Arm
        boolean eaLimits = variables.eaLimits;
        boolean eaCorrection = variables.eaCorrection;
        int eaLimitHigh = variables.eaLimitHigh;
        int eaLimitLow = variables.eaLimitLow;
        // starting POS
        boolean sp = variables.sp;
        int taSP = variables.taSP;
        int eaSP = variables.eaSP;
        // preset locations
        int specimenLoc = variables.specimenLoc;
        int submersalLoc = variables.submersalLoc;

        // Reverse one side of the drive train.
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        // breaks
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // starting pos
        if (sp) {
            // turnArm
            turnArm.setTargetPosition(taSP);
            turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // extendArm
            extendArm1.setTargetPosition(eaSP);
            extendArm2.setTargetPosition(eaSP);
            extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        int taCpos = 0;
        int eaCpos1 = extendArm1.getCurrentPosition();
        int eaCpos2 = extendArm2.getCurrentPosition();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                int taPOS = turnArm.getCurrentPosition();
                int eaPOS = extendArm1.getCurrentPosition();
                // left wheels
                leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                // right wheels
                rightFrontDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                rightBackDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                // turnArm code
                if (taLimits) {
                    if (gamepad1.right_stick_y > 0) {
                        turnArm.setTargetPosition(taLimitHigh);
                        turnArm.setPower(1);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        taCpos = turnArm.getCurrentPosition();
                    } else if (gamepad1.right_stick_y < 0) {
                        turnArm.setTargetPosition(taLimitLow);
                        turnArm.setPower(1);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        taCpos = turnArm.getCurrentPosition();
                    } else if (taCorrection) {
                        turnArm.setTargetPosition(taCpos);
                        turnArm.setPower(1);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(0);
                    }
                } else {
                    if (gamepad1.right_stick_y > 0) {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(turnArmSpeed);
                        taCpos = turnArm.getCurrentPosition();
                    } else if (gamepad1.right_stick_y < 0) {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(-turnArmSpeed);
                        taCpos = turnArm.getCurrentPosition();
                    } else if (taCorrection) {
                        turnArm.setTargetPosition(taCpos);
                        turnArm.setPower(1);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(0);
                    }
                }
                // extendArm code
                if (eaLimits) {
                    if (gamepad1.right_bumper) {
                        extendArm1.setTargetPosition(eaLimitHigh);
                        extendArm2.setTargetPosition(eaLimitHigh);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        eaCpos1 = extendArm1.getCurrentPosition();
                        eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (gamepad1.left_bumper) {
                        extendArm1.setTargetPosition(eaLimitLow);
                        extendArm2.setTargetPosition(eaLimitLow);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        eaCpos1 = extendArm1.getCurrentPosition();
                        eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (eaCorrection) {
                        extendArm1.setTargetPosition(eaCpos1);
                        extendArm2.setTargetPosition(eaCpos2);
                        extendArm1.setPower(1);
                        extendArm2.setPower(1);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(0);
                        extendArm2.setPower(0);
                    }
                } else {
                    if (gamepad1.right_bumper) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(turnArmSpeed);
                        extendArm2.setPower(turnArmSpeed);
                        eaCpos1 = extendArm1.getCurrentPosition();
                        eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (gamepad1.right_stick_y < 0) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(-turnArmSpeed);
                        extendArm2.setPower(-turnArmSpeed);
                        eaCpos1 = extendArm1.getCurrentPosition();
                        eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (taCorrection) {
                        extendArm1.setTargetPosition(eaCpos1);
                        extendArm2.setTargetPosition(eaCpos2);
                        extendArm1.setPower(1);
                        extendArm2.setPower(1);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(0);
                        extendArm2.setPower(0);
                    }
                }
                // preset code
                // specimen pos
                if (gamepad1.dpad_up) {
                    // use correction code cuz its easier fr fr
                    taCpos = specimenLoc;
                }
                // submersal pos
                if (gamepad1.dpad_down) {
                    // use correction code cuz its easier fr fr
                    taCpos = submersalLoc;
                }
                // odometry
                if (!moving) {
                    // odometry fixing itself
                }
                // telemetry
                telemetry.addData("leftBackDrive POS:", leftBackDrive.getCurrentPosition());
                telemetry.addData("leftFrontDrive POS:", leftFrontDrive.getCurrentPosition());
                telemetry.addData("rightBackDrive POS:", rightBackDrive.getCurrentPosition());
                telemetry.addData("rightFrontDrive POS:", rightFrontDrive.getCurrentPosition());
                telemetry.addData("Turn Arm Position:", taPOS);
                telemetry.addData("Turn Arm correction Position:", taCpos);
                telemetry.addData("Extend Arm Position1:", extendArm1.getCurrentPosition());
                telemetry.addData("Extend Arm Position2:", extendArm2.getCurrentPosition());
                telemetry.addData("Manually moving robot?", moving);
                telemetry.update();
            }
        }
    }
}
