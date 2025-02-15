/***
 * MAIN V3
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v3
 * started recoding at 2/14/25  @  8:31 pm
 * robot v3 expected to be finished building by 2/20/25
 */
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config("MainV3")
@TeleOp(name="Main v3", group=".ftc23403")
public class MainV3 extends LinearOpMode {
    /**
     * @TODO finish preset positions
     * @TODO have odometry working in teleOp
     * @TODO make the slides correction more smooth
     * @TODO have wrist and claw for both arms working
     * MAIN V3 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    public static double wristCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double clawCpos = 0.47;
    // claw max pos 0.3 to 0.54
    // 0.48 is the close/tight pos
    // 0.54 is close
    // 0.3 is open pos
    // 0.47 is perfect pos
    public static int eaCpos1 = 0;
    public static int eaCpos2 = 0;
    public static int saCpos1 = 0;
    public static int saCpos2 = 0;
    // misc
    public static double extendArmSpeed = 1;
    public static double submersibleArmSpeed = 1;
    public static double wheelSpeed = 1;
    // extend arm
    public static boolean eaLimits = false;
    public static boolean eaCorrection = true;
    public static int eaLimitHigh1 = 2272;
    public static int eaLimitHigh2 = 2272;
    public static int eaLimitLow1 = 0;
    public static int eaLimitLow2 = 0;
    // submersible arm
    public static boolean saLimits = false;
    public static boolean saCorrection = true;
    public static int saLimitHigh1 = 2272;
    public static int saLimitHigh2 = 2272;
    public static int saLimitLow1 = 0;
    public static int saLimitLow2 = 0;
    // preset locations
    // humanPlayer
    public static int humanPlayerLoc1 = 200;
    public static int humanPlayerLoc2 = 200;
    // baskets
    public static int basketsLoc1 = 1000;
    public static int basketsLoc2 = 1000;
    // specimens
    public static int specimenLoc1 = 717;
    public static int specimenLoc2 = 717;
    // submersible
    public static int submersibleLoc1 = 0;
    public static int submersibleLoc2 = 0;

    @Override
    public void runOpMode() {
        // hardware
        IMU imu = hardwareMap.get(IMU.class, "imu");
        Blinker control_Hub = hardwareMap.get(Blinker.class, "control_Hub");
        Blinker expansion_Hub_2 = hardwareMap.get(Blinker.class, "expansion_Hub_2");
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        // motors
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        DcMotor submersibleArm1 = hardwareMap.dcMotor.get("SubArm1");
        DcMotor submersibleArm2 = hardwareMap.dcMotor.get("SubArm2");
        // servos
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wrist = hardwareMap.get(Servo.class, "wrist");
        CRServo intake1 = hardwareMap.get(CRServo.class, "intakeL");
        CRServo intake2 = hardwareMap.get(CRServo.class, "intakeR");
        // reverse motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        submersibleArm2.setDirection(DcMotor.Direction.REVERSE);
        // wrist.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        // claw.scaleRange(0.3, 0.54);
        // breaks
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        submersibleArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        submersibleArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // misc
        imu.resetYaw();
        control_Hub.setConstant(16711927); // use http://www.shodor.org/~efarrow/trunk/html/rgbint.html for colorInt converter
        expansion_Hub_2.setConstant(16711927); // use http://www.shodor.org/~efarrow/trunk/html/rgbint.html for colorInt converter
        if (gamepad1.type.equals(Gamepad.Type.SONY_PS4)) {
            gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);
        }
        if (gamepad2.type.equals(Gamepad.Type.SONY_PS4)) {
            gamepad2.setLedColor(0, 0, 255, Integer.MAX_VALUE);
        }
        // starting pos
        eaCpos1 = extendArm1.getCurrentPosition();
        saCpos1 = submersibleArm1.getCurrentPosition();
        eaCpos2 = extendArm2.getCurrentPosition();
        saCpos2 = submersibleArm2.getCurrentPosition();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // variables
                boolean redSide = true;
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                // misc
                wrist.setPosition(wristCpos);
                claw.setPosition(clawCpos);
                if (gamepad1.share || gamepad2.share) {
                    redSide = redSide ? false : true;
                }
                // left wheels
                leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                // right wheels
                rightFrontDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                rightBackDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                // extendArm code
                if (eaLimits) {
                    if (gamepad1.dpad_up) {
                        extendArm1.setTargetPosition(eaLimitHigh1);
                        extendArm2.setTargetPosition(eaLimitHigh2);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        eaCpos1 = extendArm1.getCurrentPosition();
                        eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (gamepad1.dpad_down) {
                        extendArm1.setTargetPosition(eaLimitLow1);
                        extendArm2.setTargetPosition(eaLimitLow2);
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
                    if (gamepad1.dpad_up) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        eaCpos1 = extendArm1.getCurrentPosition();
                        eaCpos2 = extendArm2.getCurrentPosition();
                    } else if (gamepad1.dpad_down) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(-extendArmSpeed);
                        extendArm2.setPower(-extendArmSpeed);
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
                }
                // submersibleArm code
                if (saLimits) {
                    if (gamepad1.dpad_right) {
                        submersibleArm1.setTargetPosition(saLimitHigh1);
                        submersibleArm2.setTargetPosition(saLimitHigh2);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        saCpos1 = submersibleArm1.getCurrentPosition();
                        saCpos2 = submersibleArm2.getCurrentPosition();
                    } else if (gamepad1.dpad_left) {
                        submersibleArm1.setTargetPosition(saLimitLow1);
                        submersibleArm2.setTargetPosition(saLimitLow2);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        saCpos1 = submersibleArm1.getCurrentPosition();
                        saCpos2 = submersibleArm2.getCurrentPosition();
                    } else if (saCorrection) {
                        submersibleArm1.setTargetPosition(saCpos1);
                        submersibleArm2.setTargetPosition(saCpos2);
                        submersibleArm1.setPower(1);
                        submersibleArm2.setPower(1);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(0);
                        submersibleArm2.setPower(0);
                    }
                } else {
                    if (gamepad1.dpad_right) {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        saCpos1 = submersibleArm1.getCurrentPosition();
                        saCpos2 = submersibleArm2.getCurrentPosition();
                    } else if (gamepad1.dpad_left) {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(-submersibleArmSpeed);
                        submersibleArm2.setPower(-submersibleArmSpeed);
                        saCpos1 = submersibleArm1.getCurrentPosition();
                        saCpos2 = submersibleArm2.getCurrentPosition();
                    } else if (eaCorrection) {
                        submersibleArm1.setTargetPosition(saCpos1);
                        submersibleArm2.setTargetPosition(saCpos2);
                        submersibleArm1.setPower(1);
                        submersibleArm2.setPower(1);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    } else {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(0);
                        submersibleArm2.setPower(0);
                    }
                }
                // preset code
                // submersible pos
                if (gamepad1.a) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = submersibleLoc1;
                    eaCpos2 = submersibleLoc2;
                }
                // baskets pos
                if (gamepad1.y) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = basketsLoc1;
                    eaCpos2 = basketsLoc2;
                }
                // humanPlayer pos
                if (gamepad1.x) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = humanPlayerLoc1;
                    eaCpos2 = humanPlayerLoc2;
                }
                // specimen pos
                if (gamepad1.b) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = specimenLoc1;
                    eaCpos2 = specimenLoc2;
                }
                // claw
                if (gamepad1.left_trigger > 0) {
                    clawCpos = 0.47;
                } else if (gamepad1.right_trigger > 0) {
                    clawCpos = 0.54;
                }
                // odometry self-correction
                if (!moving) {
                    // odometry self-correction code
                    control_Hub.setConstant(65535); // use http://www.shodor.org/~efarrow/trunk/html/rgbint.html for colorInt converter
                    expansion_Hub_2.setConstant(65535); // use http://www.shodor.org/~efarrow/trunk/html/rgbint.html for colorInt converter
                }
                // color sensor code
                if (redSide) { //                       red                                                              yellow                                         not picked up
                    if (((sensor.red() > 20 && sensor.green() > 10 && sensor.blue() > 10) || (sensor.red() > 30 && sensor.green() > 30 && sensor.blue() > 0)) && sensor.getDistance(DistanceUnit.MM) > 1) {
                        intake1.setPower(-1);
                        intake2.setPower(1);
                    } else {
                        intake1.setPower(0);
                        intake2.setPower(0);
                    }
                } else { //                            blue                                                              yellow                                         not picked up
                    if (((sensor.red() > 10 && sensor.green() > 10 && sensor.blue() > 20) || (sensor.red() > 30 && sensor.green() > 30 && sensor.blue() > 0)) && sensor.getDistance(DistanceUnit.MM) > 1) {
                        intake1.setPower(-1);
                        intake2.setPower(1);
                    } else {
                        intake1.setPower(0);
                        intake2.setPower(0);
                    }
                }
                // telemetry
                telemetry.addData("Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Sensor RGBA:", sensor.red() + sensor.green() + sensor.blue() + sensor.alpha());
                telemetry.addData("debug:", true);
                telemetry.addData("debug:", true);
                telemetry.addData("Extend Arm Position1:", extendArm1.getCurrentPosition());
                telemetry.addData("Extend Arm Position2:", extendArm2.getCurrentPosition());
                telemetry.addData("Submersible Arm Position1:", submersibleArm1.getCurrentPosition());
                telemetry.addData("Submersible Arm Position2:", submersibleArm2.getCurrentPosition());
                telemetry.addData("Wrist Position:", wrist.getPosition());
                telemetry.addData("Claw Position:", claw.getPosition());
                telemetry.addData("triggersR?", gamepad1.right_trigger);
                telemetry.addData("triggersL?", gamepad1.left_trigger);
                telemetry.addData("Manually moving robot?", moving);
                telemetry.addData("Red side?", redSide);
                telemetry.update();
            }
        }
    }
}
