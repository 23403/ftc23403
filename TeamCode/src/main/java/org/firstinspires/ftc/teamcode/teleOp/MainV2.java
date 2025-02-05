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
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.variables.ConfigVariables;

@TeleOp(name="Main v2", group="ftc23403")
public class MainV2 extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    /**
     * @TODO add wrist and arm code
     * @TODO have odometry working in teleOp
     * @TODO make the slides correction more smooth
     * MAIN V2 BY DAVID
     * @author David Grieas - 23403 C{}de C<>nduct<>rs
     */
    @Override
    public void runOpMode() {
        // hardware
        if (gamepad1.type.equals(Gamepad.Type.SONY_PS4)) {
            gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);
        }
        if (gamepad2.type.equals(Gamepad.Type.SONY_PS4)) {
            gamepad2.setLedColor(0, 208, 255, Integer.MAX_VALUE);
        }
        // motors
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        DcMotor turnArm = hardwareMap.dcMotor.get("TurnArm");
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        // servos
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        CRServo intake1 = hardwareMap.get(CRServo.class, "intakeL");
        CRServo intake2 = hardwareMap.get(CRServo.class, "intakeR");
        // reverse motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        // wrist.setDirection(Servo.Direction.REVERSE);
        // claw.setDirection(Servo.Direction.REVERSE);
        claw.scaleRange(0.574, 0.6);
        // breaks
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // starting pos
        ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
        ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
        ConfigVariables.taCpos = ConfigVariables.taSP;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // variables
                double wheelSpeed = ConfigVariables.wheelSpeed;
                double extendArmSpeed = ConfigVariables.extendArmSpeed;
                double turnArmSpeedM = ConfigVariables.turnArmSpeed;
                boolean tightClaw = false;
                // Turn Arm
                boolean taLimits = ConfigVariables.taLimits;
                boolean taCorrection = ConfigVariables.taCorrection;
                int taLimitHigh = ConfigVariables.taLimitHigh;
                int taLimitLow = ConfigVariables.taLimitLow;
                // Extend Arm
                boolean eaLimits = ConfigVariables.eaLimits;
                boolean eaCorrection = ConfigVariables.eaCorrection;
                int eaLimitHigh1 = ConfigVariables.eaLimitHigh1;
                int eaLimitHigh2 = ConfigVariables.eaLimitHigh2;
                int eaLimitLow1 = ConfigVariables.eaLimitLow1;
                int eaLimitLow2 = ConfigVariables.eaLimitLow2;
                // preset locations
                int specimenLoc = ConfigVariables.specimenLoc;
                int submersalLoc = ConfigVariables.submersalLoc;
                int feildLoc = ConfigVariables.feildLoc;
                int basketLoc = ConfigVariables.basketLoc;
                // positions
                int taCpos = ConfigVariables.taCpos;
                int eaCpos1 = ConfigVariables.eaCpos1;
                int eaCpos2 = ConfigVariables.eaCpos2;
                double wristCpos = ConfigVariables.wristCpos;
                double clawCpos = ConfigVariables.clawCpos;
                wrist.setPosition(wristCpos);
                claw.setPosition(clawCpos);
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                double turnArmSpeed = (Math.abs(gamepad2.right_stick_y) > turnArmSpeedM) ? turnArmSpeedM : Math.abs(gamepad2.left_stick_y); // will ALWAYS return POSITIVE value!
                int taPOS = turnArm.getCurrentPosition();
                // left wheels
                leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                // right wheels
                rightFrontDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                rightBackDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                // turnArm code
                if (taLimits) {
                    if (gamepad2.right_stick_y < 0) {
                        turnArm.setTargetPosition(taLimitHigh);
                        turnArm.setPower(turnArmSpeed);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ConfigVariables.taCpos = turnArm.getCurrentPosition();
                    } else if (gamepad2.right_stick_y > 0) {
                        turnArm.setTargetPosition(taLimitLow);
                        turnArm.setPower(turnArmSpeed);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ConfigVariables.taCpos = turnArm.getCurrentPosition();
                    } else if (taCorrection) {
                        turnArm.setTargetPosition(taCpos);
                        turnArm.setPower(1);
                        turnArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(0);
                    }
                } else {
                    if (gamepad2.right_stick_y > 0) {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(turnArmSpeed);
                        ConfigVariables.taCpos = turnArm.getCurrentPosition();
                    } else if (gamepad2.right_stick_y < 0) {
                        turnArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        turnArm.setPower(-turnArmSpeed);
                        ConfigVariables.taCpos = turnArm.getCurrentPosition();
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
                    if (gamepad2.dpad_up) {
                        extendArm1.setTargetPosition(eaLimitHigh1);
                        extendArm2.setTargetPosition(eaLimitHigh2);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
                        ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (gamepad2.dpad_down) {
                        extendArm1.setTargetPosition(eaLimitLow1);
                        extendArm2.setTargetPosition(eaLimitLow2);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
                        ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
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
                    if (gamepad2.dpad_up) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
                        ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (gamepad2.dpad_down) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(-extendArmSpeed);
                        extendArm2.setPower(-extendArmSpeed);
                        ConfigVariables.eaCpos1 = extendArm1.getCurrentPosition();
                        ConfigVariables.eaCpos2 = extendArm2.getCurrentPosition();
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
                }
                // preset code
                // field pos
                if (gamepad2.a) {
                    // use correction code cuz its easier fr fr
                    taCpos = feildLoc;
                    wristCpos = 0.12;
                }
                // baskets pos
                if (gamepad2.y) {
                    // use correction code cuz its easier fr fr
                    taCpos = basketLoc;
                    wristCpos = 0.12;
                }
                // specimen pos
                if (gamepad2.b) {
                    // use correction code cuz its easier fr fr
                    taCpos = specimenLoc;
                    wristCpos = 0.09;
                }
                // submersal pos
                if (gamepad2.x) {
                    // use correction code cuz its easier fr fr
                    taCpos = submersalLoc;
                    wristCpos = 0.5;
                }
                // intake
                if (gamepad1.right_bumper) {
                    intake1.setPower(-1);
                    intake2.setPower(1);
                } else if (gamepad1.left_bumper) {
                    intake1.setPower(1);
                    intake2.setPower(-1);
                } else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                }
                // claw
                if (gamepad1.right_trigger > 0) {
                    claw.setPosition(0);
                    tightClaw = false;
                } else if (gamepad1.left_trigger > 0) {
                    if (!tightClaw) {
                        claw.setPosition(0.25);
                        tightClaw = true;
                    } else {
                        claw.setPosition(0.35);
                        tightClaw = false;
                    }
                }
                 // odometry self-correction
                if (!moving) {
                    // odometry self-correction code
                }
                // telemetry
                telemetry.addData("leftBackDrive POS:", leftBackDrive.getCurrentPosition());
                telemetry.addData("leftFrontDrive POS:", leftFrontDrive.getCurrentPosition());
                telemetry.addData("rightBackDrive POS:", rightBackDrive.getCurrentPosition());
                telemetry.addData("rightFrontDrive POS:", rightFrontDrive.getCurrentPosition());
                telemetry.addData("Turn Arm Position:", taPOS);
                telemetry.addData("Turn Arm correction Position:", taCpos);
                telemetry.addData("Turn Arm Speed", turnArmSpeed);
                telemetry.addData("Extend Arm Position1:", extendArm1.getCurrentPosition());
                telemetry.addData("Extend Arm Position2:", extendArm2.getCurrentPosition());
                telemetry.addData("Wrist Position:", wrist.getPosition());
                telemetry.addData("Claw Position:", claw.getPosition());
                telemetry.addData("tightClaw?", tightClaw);
                telemetry.addData("triggersR?", gamepad1.right_trigger);
                telemetry.addData("triggersL?", gamepad1.left_trigger);
                telemetry.addData("Manually moving robot?", moving);
                telemetry.update();
            }
        }
    }
}
