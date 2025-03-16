/***
 * MAIN V4
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v4
 * started recoding at 3/4/25  @  5:34 pm
 * robot v4 finished building at 3/14/25
 */
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import xyz.nin1275.constants.PIDConstants;
import xyz.nin1275.controllers.PIDController;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.GamepadUtils;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Sensor;
import xyz.nin1275.utils.Timer;

@Config("MainV4")
@TeleOp(name="Main v4", group=".ftc23403")
public class MainV4 extends LinearOpMode {
    /**
     * @TODO have odometry driving working
     * @TODO make the fucking slides smooth
     * MAIN V4 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    // servos
    public static double wristCpos1 = 0.38;
    // 0.5 low pos
    // 0.38 grab from sa
    // 1 high pos
    public static double clawCpos1 = 0.9;
    // 0.4 is close
    // 0.9 is open
    public static double sweeperCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double wristCpos2 = 0.45;
    // 0.1 low pos
    // 0.45 give to ea
    // 0.6 high pos
    public static double clawCpos2 = 0.5;
    // 0.5 is close
    // 0.55 is grab pos
    // 1 is open pos
    public static double armCpos = 0.72;
    // 0.88 low pos
    // 0.72 grab from sa
    // 0 high pos
    public static double subArmCpos = 0.45;
    // 0 low pos
    // 0.45 high pos
    // corrections
    public static int eaCpos1 = 0;
    public static int eaCpos2 = 0;
    // misc
    private static boolean ran = false;
    public static boolean redSide = true;
    public static int extendArmSpeed = 300;
    public static double submersibleArmSpeed = 0.01;
    public static double wheelSpeed = 1;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static boolean eaLimits = false;
    public static int eaLimitHigh1 = 5283;
    public static int eaLimitHigh2 = 5082;
    public static int eaLimitLow1 = 1;
    public static int eaLimitLow2 = 5;
    // pid stuff
    @Config("PID Tuning")
    public static class PID {
        public static boolean tunePID = true;
        public static boolean moveTestPID = false;
        public static int target = 0;
        public static double tickPerRevolution = 751.8 / 180;
    }
    @Override
    public void runOpMode()  {
        // hardware
        IMU imu = hardwareMap.get(IMU.class, "imu");
        MetroLib.setConstants(MConstants.class);
        Follower follower = new Follower(hardwareMap);
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower, sensor);
        PIDController controller = new PIDController(0, 0);
        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);
        GamepadEx gamepad2Ex = new GamepadEx(gamepad2);
        // motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        /*
        DcMotorEx submersibleArm1 = hardwareMap.get(DcMotorEx.class, "SubArm1");
        DcMotorEx submersibleArm2 = hardwareMap.get(DcMotorEx.class, "SubArm2");
        */
        // servos
        Servo sweeper = hardwareMap.get(Servo.class, "sweeper"); // goBilda torque
        // ea
        Servo arm = hardwareMap.get(Servo.class, "arm"); // axon
        /*
        Servo arm1 = hardwareMap.get(Servo.class, "arm1"); // axon
        Servo arm2 = hardwareMap.get(Servo.class, "arm2"); // axon
        */
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // axon
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // axon
        // sa
        Servo submersibleArm = hardwareMap.get(Servo.class, "subArm"); // axon
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 20kg
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // axon
        CRServo intake1 = hardwareMap.get(CRServo.class, "intakeL"); // goBilda speed
        CRServo intake2 = hardwareMap.get(CRServo.class, "intakeR"); // goBilda speed
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        sweeper.setDirection(Servo.Direction.REVERSE);
        // breaks
        Motors.setBrakes(List.of(leftFront, rightFront, leftRear, rightRear));
        // misc
        GamepadUtils.setGamepad1Color(0, 255, 0, Integer.MAX_VALUE);
        GamepadUtils.setGamepad2Color(255, 0, 255, Integer.MAX_VALUE);
        extendArm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // calibration
        imu.resetYaw();
        if (MConstants.startUp) {
            Calibrate.TeleOp.saveStartPositions(List.of(extendArm1, extendArm2));
            MConstants.startUp = false;
        }
        Calibrate.TeleOp.calibrateStartup(List.of(extendArm1, extendArm2));
        follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        Calibrate.Auto.clearEverything();
        // starting pos
        eaCpos1 = extendArm1.getCurrentPosition();
        // saCpos1 = submersibleArm1.getCurrentPosition();
        eaCpos2 = extendArm2.getCurrentPosition();
        // saCpos2 = submersibleArm2.getCurrentPosition();
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
                arm.setPosition(armCpos);
                // arm1.setPosition(armCpos1);
                // arm2.setPosition(armCpos2);
                submersibleArm.setPosition(subArmCpos);
                sweeper.setPosition(sweeperCpos);
                if (gamepad1Ex.wasJustPressed(GamepadKeys.Button.START) || gamepad2Ex.wasJustPressed(GamepadKeys.Button.START)) redSide = !redSide;
                // movements
                if (!odoDrive) {
                    follower.breakFollowing();
                    // drive
                    double forward = -gamepad1.left_stick_y; // forward
                    double strafe = gamepad1.left_stick_x; // strafe
                    double turn = gamepad1.right_stick_x;  // rotation
                    // formula
                    double leftFrontPower = (forward + strafe + turn) * wheelSpeed;
                    double leftBackPower = (forward - strafe + turn) * wheelSpeed;
                    double rightFrontPower = -(forward - strafe - turn) * wheelSpeed;
                    double rightBackPower = (forward + strafe - turn) * wheelSpeed;
                    // power
                    leftFront.setPower(leftFrontPower);
                    leftRear.setPower(leftBackPower);
                    rightFront.setPower(rightFrontPower);
                    rightRear.setPower(rightBackPower);
                } else {
                    follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);
                    follower.startTeleopDrive();
                }
                // extendArm code
                int eaPos1 = extendArm1.getCurrentPosition();
                int eaPos2 = extendArm2.getCurrentPosition();
                double ff1 = Math.cos(Math.toRadians(-eaPos1/ PID.tickPerRevolution)) * PIDConstants.F;
                double ff2 = Math.cos(Math.toRadians(-eaPos2/ PID.tickPerRevolution)) * PIDConstants.F;
                // formula
                double pid1 = controller.calculate(eaPos1, eaCpos1);
                double pid2 = controller.calculate(eaPos2, eaCpos2);
                double power1 = pid1 + ff1;
                double power2 = pid2 + ff2;
                // movement
                extendArm1.setPower(power1);
                extendArm2.setPower(power2);
                // controls
                if (gamepad2.dpad_up) {
                    if (eaLimits) {
                        if (eaCpos1 < eaLimitHigh1 || eaCpos2 < eaLimitHigh2) {
                            eaCpos1 += extendArmSpeed;
                            eaCpos2 += extendArmSpeed;
                        }
                    } else {
                        eaCpos1 += extendArmSpeed;
                        eaCpos2 += extendArmSpeed;
                    }
                } else if (gamepad2.dpad_down) {
                    if (eaLimits) {
                        if (eaCpos1 > eaLimitLow1 || eaCpos2 > eaLimitLow2) {
                            eaCpos1 -= extendArmSpeed;
                            eaCpos2 -= extendArmSpeed;
                        }
                    } else {
                        eaCpos1 -= extendArmSpeed;
                        eaCpos2 -= extendArmSpeed;
                    }
                }
                // submersibleArm code
                if (gamepad1.dpad_up) {
                    subArmCpos = subArmCpos < 0.45 ? 0.45 : subArmCpos - submersibleArmSpeed;
                } else if (gamepad1.dpad_down) {
                    subArmCpos = subArmCpos > 1 ? 1 : subArmCpos + submersibleArmSpeed;
                }
                // pid test
                if (PID.tunePID) {
                    // initialization
                    extendArm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extendArm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    // formula
                    pid1 = controller.calculate(eaPos1, PID.target);
                    pid2 = controller.calculate(eaPos2, PID.target);
                    power1 = pid1 + ff1;
                    power2 = pid2 + ff2;
                    // movement
                    extendArm1.setPower(power1);
                    extendArm2.setPower(power2);
                    // telemetry
                    telemetry.addData("TARGET", PID.target);
                    telemetry.addData("PID POS1", eaPos1);
                    telemetry.addData("PID POS2", eaPos2);
                    telemetry.update();
                } else if (PID.moveTestPID) {
                    // initialization
                    extendArm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    extendArm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    // movement
                    if (extendArm1.getCurrentPosition() > 2900 || extendArm2.getCurrentPosition() > 2900) {
                        pid1 = controller.calculate(eaPos1, 0);
                        pid2 = controller.calculate(eaPos2, 0);
                        power1 = pid1 + ff1;
                        power2 = pid2 + ff2;
                        extendArm1.setPower(power1);
                        extendArm2.setPower(power2);
                    } else if (extendArm1.getCurrentPosition() < 100 || extendArm2.getCurrentPosition() < 100) {
                        pid1 = controller.calculate(eaPos1, 3000);
                        pid2 = controller.calculate(eaPos2, 3000);
                        power1 = pid1 + ff1;
                        power2 = pid2 + ff2;
                        extendArm1.setPower(power1);
                        extendArm2.setPower(power2);
                    }
                    // telemetry
                    telemetry.addData("PID POS1", eaPos1);
                    telemetry.addData("PID POS2", eaPos2);
                    telemetry.update();
                }
                // preset code
                /**
                 * GAMEPAD 1
                 *   X / ▢         - Score Specimen Preset
                 *   Y / Δ         - EMPTY
                 *   B / O         - Grab from Human Player Preset
                 *   A / X         - Grab from Submersible Preset
                 *
                 *                    ________
                 *                   / ______ \\
                 *     ------------.-'   _  '-..+
                 *              /   _  ( Y )  _  \\
                 *             |  ( X )  _  ( B ) |
                 *        ___  '.      ( A )     /|
                 *      .'    '.    '-._____.-'  .'
                 *     |       |                 |
                 *      '.___.' '.               |
                 *               '.             /
                 *                \\.          .'
                 *                  \\________/
                */
                // specimen pos
                if (gamepad1.x) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 850;
                    eaCpos2 = 850;
                    armCpos = 0.4;
                    wristCpos1 = 0.8;
                    clawCpos1 = 0.4;
                }
                // humanPlayer pos
                if (gamepad1.b) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 0;
                    eaCpos2 = 0;
                    subArmCpos = 0;
                    armCpos = 0.01;
                    wristCpos1 = 0.33;
                    clawCpos1 = 0.9;
                }
                // submersible pos
                if (gamepad1.a) {
                    // use correction code cuz its easier fr fr
                    subArmCpos = 0.45;
                    if (armCpos > 0.7) {
                        armCpos = 0.7;
                    }
                    wristCpos2 = 0.1;
                    clawCpos2 = 0.55;
                }
                /**
                 * GAMEPAD 2
                 *   X / ▢         - Transition from Submersible arm to Extend arm
                 *   Y / Δ         - Place in High Basket
                 *   B / O         - Place in Low basket
                 *   A / X         - EMPTY
                 *
                 *                    ________
                 *                   / ______ \\
                 *     ------------.-'   _  '-..+
                 *              /   _  ( Y )  _  \\
                 *             |  ( X )  _  ( B ) |
                 *        ___  '.      ( A )     /|
                 *      .'    '.    '-._____.-'  .'
                 *     |       |                 |
                 *      '.___.' '.               |
                 *               '.             /
                 *                \\.          .'
                 *                  \\________/
                 */
                // high basket pos
                if (gamepad2.y) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 5300;
                    eaCpos2 = 5200;
                    wristCpos1 = 1;
                    clawCpos1 = 0.4;
                    armCpos = 0.2;
                }
                // low basket pos
                if (gamepad2.b) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 2700;
                    eaCpos2 = 2600;
                    wristCpos1 = 1;
                    clawCpos1 = 0.4;
                    armCpos = 0.2;
                }
                // transition pos
                if (gamepad2.x) {
                    // use correction code cuz its easier fr fr
                    eaCpos1 = 0;
                    eaCpos2 = 0;
                    subArmCpos = 0;
                    clawCpos2 = 0.55;
                    wristCpos2 = 0.48;
                    wristCpos1 = 0.7;
                    clawCpos1 = 0.9;
                    armCpos = 0.74;
                }
                // claws
                if (gamepad2.left_trigger > 0) {
                    clawCpos1 = 0.9;
                } else if (gamepad2.right_trigger > 0) {
                    clawCpos1 = 0.4;
                }
                if (gamepad1.left_trigger > 0) {
                    clawCpos2 = 0.55;
                } else if (gamepad1.right_trigger > 0) {
                    clawCpos2 = 0.5;
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
                if ((redSide ? Sensor.pickUpRed() : Sensor.pickUpBlue() || Sensor.pickUpYellow()) || gamepad1.right_bumper) {
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
                Calibrate.TeleOp.getStartPositions();
                telemetry.addData("DEBUG:", "PickUp " + (Sensor.pickUpRed() ? "RED" : Sensor.pickUpBlue() ? "BLUE" : Sensor.pickUpYellow() ? "YELLOW" : "NONE"));
                telemetry.addData("DEBUG:", "Grabbed " + (Sensor.isRedGrabbed() ? "RED" : Sensor.isBlueGrabbed() ? "BLUE" : Sensor.isYellowGrabbed() ? "YELLOW" : "NONE"));
                telemetry.addData("Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
                telemetry.addData("Extend Arm Position1:", extendArm1.getCurrentPosition());
                telemetry.addData("Extend Arm Position2:", extendArm2.getCurrentPosition());
                telemetry.addData("Submersible Arm Position:", submersibleArm.getPosition());
                telemetry.addData("Wrist Position1:", wrist1.getPosition());
                telemetry.addData("Wrist Position2:", wrist2.getPosition());
                telemetry.addData("Claw Position1:", claw1.getPosition());
                telemetry.addData("Claw Position2:", claw2.getPosition());
                telemetry.addData("Arm Position:", arm.getPosition());
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
