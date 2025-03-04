/***
 * MAIN V4
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v4
 * started recoding at 3/4/25  @  5:34 pm
 * robot v4 finished building at 2/5/25
 */
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import xyz.nin1275.Calibrate;
import xyz.nin1275.GamepadUtils;
import xyz.nin1275.MetroLib;
import xyz.nin1275.Motors;
import xyz.nin1275.Sensor;
import xyz.nin1275.Timer;

@Config("MainV4")
@TeleOp(name="Main v4", group=".ftc23403")
public class MainV4 extends LinearOpMode {
    /**
     * @TODO have odometry driving working
     * MAIN V4 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    // servos
    public static double wristCpos1 = 0.5;
    // 0.5 low pos
    // 0.7 grab from sa
    // 1 high pos
    public static double clawCpos1 = 0.9;
    // 0.4 is close
    // 0.9 is open
    public static double sweeperCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double wristCpos2 = 0.6;
    // 0.1 low pos
    // 0.45 give to ea
    // 0.6 high pos
    public static double clawCpos2 = 0.55;
    // 0.5 is close
    // 0.55 is grab pos
    // 1 is open pos
    public static double armCpos1 = 0;
    // 0.88 low pos
    // 0.75 grab from sa
    // 0 high pos
    public static double armCpos2 = 0;
    // 0.88 low pos
    // 0.75 grab from sa
    // 0 high pos
    // corrections
    public static int eaCpos1 = 0;
    public static int eaCpos2 = 0;
    public static int saCpos1 = 0;
    public static int saCpos2 = 0;
    // misc
    private static boolean ran = false;
    public static boolean redSide = true;
    public static double extendArmSpeed = 1;
    public static double submersibleArmSpeed = 1;
    public static double wheelSpeed = 1;
    // odometry
    public static double X = 0;
    public static double Y = 0;
    public static double HEADING;
    public static boolean selfCorrection = false;
    public static boolean odoDrive = false;
    // extend arm
    public static boolean eaLimits = false;
    public static boolean eaCorrection = true;
    public static int eaLimitHigh1 = 2272;
    public static int eaLimitHigh2 = 2272;
    public static int eaLimitLow1 = 0;
    public static int eaLimitLow2 = 0;
    public static int eaErrorCorr = 100;
    // submersible arm
    public static boolean saLimits = false;
    public static boolean saCorrection = true;
    public static int saLimitHigh1 = 2272;
    public static int saLimitHigh2 = 2272;
    public static int saLimitLow1 = 0;
    public static int saLimitLow2 = 0;
    public static int saErrorCorr = 100;
    @Override
    public void runOpMode()  {
        // hardware
        IMU imu = hardwareMap.get(IMU.class, "imu");
        MetroLib.setConstants(MConstants.class);
        Follower follower = new Follower(hardwareMap);
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower, sensor);
        // motors
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        DcMotor submersibleArm1 = hardwareMap.dcMotor.get("SubArm1");
        DcMotor submersibleArm2 = hardwareMap.dcMotor.get("SubArm2");
        // servos
        Servo sweeper = hardwareMap.get(Servo.class, "sweeper");
        // ea
        Servo arm1 = hardwareMap.get(Servo.class, "arm1"); // axon
        Servo arm2 = hardwareMap.get(Servo.class, "arm2"); // axon
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 25kg
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // axon
        // sa
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 20kg
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // axon
        CRServo intake1 = hardwareMap.get(CRServo.class, "intakeL"); // goBilda speed
        CRServo intake2 = hardwareMap.get(CRServo.class, "intakeR"); // goBilda speed
        // reverse motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        extendArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        submersibleArm2.setDirection(DcMotorSimple.Direction.REVERSE);
        // breaks
        Motors.setBrakes(List.of(leftFront, rightFront, leftRear, rightRear, extendArm1, extendArm2, submersibleArm1, submersibleArm2));
        Motors.resetEncoders(List.of(extendArm1, submersibleArm1, extendArm2, submersibleArm2));
        // misc
        GamepadUtils.setGamepad1Color(0, 255, 0, Integer.MAX_VALUE);
        GamepadUtils.setGamepad2Color(255, 0, 255, Integer.MAX_VALUE);
        // calibration
        imu.resetYaw();
        Calibrate.TeleOp.calibrateFromAuto(List.of(extendArm1, extendArm2, submersibleArm1, submersibleArm2));
        follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        Calibrate.Auto.clearEverything();
        // starting pos
        eaCpos1 = extendArm1.getCurrentPosition();
        saCpos1 = submersibleArm1.getCurrentPosition();
        eaCpos2 = extendArm2.getCurrentPosition();
        saCpos2 = submersibleArm2.getCurrentPosition();
        // telemetry
        telemetry.addData("RESETTING", "DONE!");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // variables
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                // misc
                wrist1.setPosition(wristCpos1);
                wrist2.setPosition(wristCpos2);
                claw1.setPosition(clawCpos1);
                claw2.setPosition(clawCpos2);
                arm1.setPosition(armCpos1);
                arm2.setPosition(armCpos2);
                sweeper.setPosition(sweeperCpos);
                if (gamepad1.share || gamepad2.share) {
                    redSide = redSide ? false : true;
                }
                // movements
                if (!odoDrive) {
                    follower.breakFollowing();
                    // drive
                    double forward = -gamepad1.left_stick_y; // forwards
                    double strafe = gamepad1.left_stick_x; // sideways
                    double turn = gamepad1.right_stick_x;  // rotation
                    // formula
                    double leftFrontPower = (forward + strafe + turn) * wheelSpeed;
                    double leftBackPower = (forward - strafe + turn) * wheelSpeed;
                    double rightFrontPower = (forward - strafe - turn) * wheelSpeed;
                    double rightBackPower = (forward + strafe - turn) * wheelSpeed;
                    // speed
                    leftFront.setPower(leftFrontPower);
                    leftRear.setPower(leftBackPower);
                    rightFront.setPower(rightFrontPower);
                    rightRear.setPower(rightBackPower);
                } else {
                    follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
                    follower.startTeleopDrive();
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
                        eaCpos1 = extendArm1.getCurrentPosition() + eaErrorCorr;
                        eaCpos2 = extendArm2.getCurrentPosition() + eaErrorCorr;
                    } else if (gamepad2.dpad_down) {
                        extendArm1.setTargetPosition(eaLimitLow1);
                        extendArm2.setTargetPosition(eaLimitLow2);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        eaCpos1 = extendArm1.getCurrentPosition() - eaErrorCorr;
                        eaCpos2 = extendArm2.getCurrentPosition() - eaErrorCorr;
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
                        eaCpos1 = extendArm1.getCurrentPosition() + eaErrorCorr;
                        eaCpos2 = extendArm2.getCurrentPosition() + eaErrorCorr;
                    } else if (gamepad2.dpad_down) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(-extendArmSpeed);
                        extendArm2.setPower(-extendArmSpeed);
                        eaCpos1 = extendArm1.getCurrentPosition() - eaErrorCorr;
                        eaCpos2 = extendArm2.getCurrentPosition() - eaErrorCorr;
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
                    if (gamepad1.dpad_up) {
                        submersibleArm1.setTargetPosition(saLimitHigh1);
                        submersibleArm2.setTargetPosition(saLimitHigh2);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        saCpos1 = submersibleArm1.getCurrentPosition() + saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() + saErrorCorr;
                    } else if (gamepad1.dpad_down) {
                        submersibleArm1.setTargetPosition(saLimitLow1);
                        submersibleArm2.setTargetPosition(saLimitLow2);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        saCpos1 = submersibleArm1.getCurrentPosition() - saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() - saErrorCorr;
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
                    if (gamepad1.dpad_up) {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(-submersibleArmSpeed);
                        submersibleArm2.setPower(-submersibleArmSpeed);
                        saCpos1 = submersibleArm1.getCurrentPosition() + saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() + saErrorCorr;
                    } else if (gamepad1.dpad_down) {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        saCpos1 = submersibleArm1.getCurrentPosition() - saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() - saErrorCorr;
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
                }
                // preset code
                // submersible pos
                if (gamepad2.a) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 0;
                    eaCpos2 = 0;
                    saCpos1 = 300;
                    saCpos2 = 300;
                    if (armCpos1 > 0.7 || armCpos2 > 0.7) {
                        armCpos1 = 0.7;
                        armCpos2 = 0.7;
                    }
                    wristCpos2 = 0.1;
                    clawCpos2 = 0.55;
                }
                // specimen pos
                if (gamepad2.b) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 0;
                    eaCpos2 = 0;
                    saCpos1 = 0;
                    saCpos2 = 0;
                    armCpos1 = 0;
                    armCpos2 = 0;
                }
                // transition pos
                if (gamepad2.y) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 0;
                    eaCpos2 = 0;
                    saCpos1 = 0;
                    saCpos2 = 0;
                    wristCpos2 = 0.45;
                    clawCpos2 = 0.5;
                    armCpos1 = 0.75;
                    armCpos2 = 0.75;
                    wristCpos1 = 0.7;
                    clawCpos1 = 0.9;
                    Timer.wait(100);
                    clawCpos2 = 0.55;
                    clawCpos1 = 0.4;
                    armCpos1 = 0.7;
                    armCpos2 = 0.7;
                }
                // humanPlayer pos
                if (gamepad2.x) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 0;
                    eaCpos2 = 0;
                    saCpos1 = 0;
                    saCpos2 = 0;
                    armCpos1 = 0;
                    armCpos2 = 0;
                    wristCpos1 = 0.7;
                    clawCpos1 = 0.9;
                }
                // claws
                if (gamepad1.left_trigger > 0) {
                    clawCpos1 = 0.55;
                } else if (gamepad1.right_trigger > 0) {
                    clawCpos1 = 0.5;
                }
                if (gamepad2.left_trigger > 0) {
                    clawCpos2 = 0.9;
                } else if (gamepad2.right_trigger > 0) {
                    clawCpos2 = 0.4;
                }
                // color sensor code
                if (Sensor.isRedGrabbed()) {
                    GamepadUtils.setGamepad1Color(255, 0, 0, Integer.MAX_VALUE);
                    GamepadUtils.setGamepad2Color(255, 0, 0, Integer.MAX_VALUE);
                    if (!ran) {
                        GamepadUtils.viberate(gamepad1, 20, 1000);
                        GamepadUtils.viberate(gamepad2, 20, 1000);
                        ran = true;
                    }
                } else if (Sensor.isBlueGrabbed()) {
                    GamepadUtils.setGamepad1Color(0, 0, 255, Integer.MAX_VALUE);
                    GamepadUtils.setGamepad2Color(0, 0, 255, Integer.MAX_VALUE);
                    if (!ran) {
                        GamepadUtils.viberate(gamepad1, 20, 1000);
                        GamepadUtils.viberate(gamepad2, 20, 1000);
                        ran = true;
                    }
                } else if (Sensor.isYellowGrabbed()) {
                    GamepadUtils.setGamepad1Color(255, 255, 0, Integer.MAX_VALUE);
                    GamepadUtils.setGamepad2Color(255, 255, 0, Integer.MAX_VALUE);
                    if (!ran) {
                        GamepadUtils.viberate(gamepad1, 20, 1000);
                        GamepadUtils.viberate(gamepad2, 20, 1000);
                        ran = true;
                    }
                } else {
                    GamepadUtils.setGamepadColorOld(1, Integer.MAX_VALUE);
                    GamepadUtils.setGamepadColorOld(2, Integer.MAX_VALUE);
                    ran = false;
                }
                // auto intake
                if (redSide ? Sensor.pickUpRed() : Sensor.pickUpBlue() || Sensor.pickUpYellow() || gamepad1.right_bumper) {
                    intake1.setPower(-1);
                    intake2.setPower(1);
                } else if (gamepad1.left_bumper) {
                    intake1.setPower(1);
                    intake2.setPower(-1);
                } else {
                    intake1.setPower(0);
                    intake2.setPower(0);
                }
                // alerts
                // 15 seconds left and start of match
                if (Timer.TeleOp.alert(15) || Timer.TeleOp.alert(150)) {
                    GamepadUtils.viberate(gamepad1, 20, 1000);
                    GamepadUtils.viberate(gamepad2, 20, 1000);
                }
                // telemetry
                telemetry.addData("Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
                telemetry.addData("Extend Arm Position1:", extendArm1.getCurrentPosition());
                telemetry.addData("Extend Arm Position2:", extendArm2.getCurrentPosition());
                telemetry.addData("Submersible Arm Position1:", submersibleArm1.getCurrentPosition());
                telemetry.addData("Submersible Arm Position2:", submersibleArm2.getCurrentPosition());
                telemetry.addData("Wrist Position1:", wrist1.getPosition());
                telemetry.addData("Wrist Position2:", wrist2.getPosition());
                telemetry.addData("Claw Position1:", claw1.getPosition());
                telemetry.addData("Claw Position2:", claw2.getPosition());
                telemetry.addData("Arm Position1:", arm1.getPosition());
                telemetry.addData("Arm Position2:", arm2.getPosition());
                telemetry.addData("Sweeper Position:", sweeper.getPosition());
                telemetry.addData("triggersR?", gamepad1.right_trigger);
                telemetry.addData("triggersL?", gamepad1.left_trigger);
                telemetry.addData("Manually moving robot?", moving);
                telemetry.addData("Red side?", redSide);
                telemetry.update();
            }
        }
    }
}
