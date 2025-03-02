/***
 * MAIN V3
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v3
 * started recoding at 2/14/25  @  8:31 pm
 * robot v3 expected to be finished building by 3/3/25
 */
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import xyz.nin1275.Servos;
import xyz.nin1275.Timer;

@Config("MainV3")
@TeleOp(name="Main v3", group=".ftc23403")
public class MainV3 extends LinearOpMode {
    /**
     * @TODO finish preset positions
     * @TODO have wrist and claw for both arms working
     * MAIN V3 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    // servos
    public static double wristCpos1 = 1;
    // 0.5 low pos
    // 0.6 grab from sa
    // 1 high pos
    public static double clawCpos1 = 0.47;
    // 1 is close
    // 0 is open
    public static double sweeperCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double wristCpos2 = 1;
    // 0.5 low pos
    // 0.6 give to ea
    // 1 high pos
    public static double clawCpos2 = 0.47;
    // 1 is close
    // 0.47 is grab pos
    // 0.48 is the tight pos
    // 0 is open pos
    public static double armCpos1 = 1;
    // 0 low pos
    // 1 high pos
    public static double armCpos2 = 1;
    // 0 low pos
    // 1 high pos
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
    public static boolean odoDrive = true;
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
        MetroLib.setConstants(MConstants.class);
        Follower follower = new Follower(hardwareMap);
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower, sensor);
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
        Motors.reverse(List.of(rightFrontDrive, rightBackDrive, extendArm2, submersibleArm2));
        // Servos.reverse(List.of());
        // positions
        // claw.scaleRange(0.3, 0.54);
        // breaks
        Motors.setBrakes(List.of(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, extendArm1, extendArm2, submersibleArm1, submersibleArm2));
        Motors.resetEncoders(List.of(extendArm1, submersibleArm1, extendArm2, submersibleArm2));
        // misc
        GamepadUtils.setGamepad1Color(0, 255, 0, Integer.MAX_VALUE);
        GamepadUtils.setGamepad2Color(255, 0, 255, Integer.MAX_VALUE);
        // calibration
        imu.resetYaw();
        Calibrate.TeleOp.calibrateFromAuto(List.of(extendArm1, extendArm2, submersibleArm1, submersibleArm2));
        // starting pos
        eaCpos1 = extendArm1.getCurrentPosition();
        saCpos1 = submersibleArm1.getCurrentPosition();
        eaCpos2 = extendArm2.getCurrentPosition();
        saCpos2 = submersibleArm2.getCurrentPosition();
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
                if (odoDrive) {
                    follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
                    follower.startTeleopDrive();
                } else {
                    follower.breakFollowing();
                    // left wheels
                    leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                    leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                    // right wheels
                    rightFrontDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                    rightBackDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y + gamepad1.right_stick_x)) * wheelSpeed);
                }
                // extendArm code
                if (eaLimits) {
                    if (gamepad1.dpad_up) {
                        extendArm1.setTargetPosition(eaLimitHigh1);
                        extendArm2.setTargetPosition(eaLimitHigh2);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        eaCpos1 = extendArm1.getCurrentPosition() + eaErrorCorr;
                        eaCpos2 = extendArm2.getCurrentPosition() + eaErrorCorr;
                    } else if (gamepad1.dpad_down) {
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
                    if (gamepad1.dpad_up) {
                        extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        extendArm1.setPower(extendArmSpeed);
                        extendArm2.setPower(extendArmSpeed);
                        eaCpos1 = extendArm1.getCurrentPosition() + eaErrorCorr;
                        eaCpos2 = extendArm2.getCurrentPosition() + eaErrorCorr;
                    } else if (gamepad1.dpad_down) {
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
                    if (gamepad1.dpad_right) {
                        submersibleArm1.setTargetPosition(saLimitHigh1);
                        submersibleArm2.setTargetPosition(saLimitHigh2);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        saCpos1 = submersibleArm1.getCurrentPosition() + saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() + saErrorCorr;
                    } else if (gamepad1.dpad_left) {
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
                    if (gamepad1.dpad_right) {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(-submersibleArmSpeed);
                        submersibleArm2.setPower(-submersibleArmSpeed);
                        saCpos1 = submersibleArm1.getCurrentPosition() + saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() + saErrorCorr;
                    } else if (gamepad1.dpad_left) {
                        submersibleArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        submersibleArm1.setPower(submersibleArmSpeed);
                        submersibleArm2.setPower(submersibleArmSpeed);
                        saCpos1 = submersibleArm1.getCurrentPosition() - saErrorCorr;
                        saCpos2 = submersibleArm2.getCurrentPosition() - saErrorCorr;
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
                    clawCpos1 = 0.47;
                } else if (gamepad1.right_trigger > 0) {
                    clawCpos1 = 0.54;
                }
                // odometry self-correction
                if (selfCorrection) {
                    if (!moving) {
                        // odometry self-correction code
                        follower.holdPoint(new Point(X, Y), HEADING);
                    } else {
                        X = follower.getPose().getX();
                        Y = follower.getPose().getY();
                        HEADING = follower.getPose().getHeading();
                    }
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
                if (redSide) {
                    if (Sensor.pickUpRed() || Sensor.pickUpYellow()) {
                        intake1.setPower(-1);
                        intake2.setPower(1);
                    } else {
                        intake1.setPower(0);
                        intake2.setPower(0);
                    }
                } else {
                    if (Sensor.pickUpBlue() || Sensor.pickUpYellow()) {
                        intake1.setPower(-1);
                        intake2.setPower(1);
                    } else {
                        intake1.setPower(0);
                        intake2.setPower(0);
                    }
                }
                // alerts
                // 15 seconds left
                if (Timer.TeleOp.alert(15)) {
                    GamepadUtils.viberate(gamepad1, 20, 1000);
                    GamepadUtils.viberate(gamepad2, 20, 1000);
                }
                // start of match
                if (Timer.TeleOp.alert(150)) {
                    GamepadUtils.viberate(gamepad1, 20, 1000);
                    GamepadUtils.viberate(gamepad2, 20, 1000);
                }
                // telemetry
                telemetry.addData("Sensor Distance IN:", sensor.getDistance(DistanceUnit.INCH));
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