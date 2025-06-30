/***
 * MAIN V4
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch for our robot, Beastkit v4
 * started recoding at 3/4/25  @  5:34 pm
 * robot v4 finished building at 3/14/25
 */
package org.firstinspires.ftc.teamcode.teleOp.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.testCode.slides.ea.PIDTuneSlides;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.variables.constants.MConstants;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Sensor;
import xyz.nin1275.utils.Timer;

@Config("MainV4")
@TeleOp(name="Main v4", group="old_ftc23403")
public class MainV4 extends LinearOpMode {
    /**
     * @TODO have odometry driving working
     * @TODO finish all the presets PROPERLY
     * @TODO get limelight in here
     * @TODO fix color sensor shit
     * MAIN V4 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    // servos
    public static double wristCpos1 = 0;
    // 0.5 low pos
    // 0.38 grab from sa
    // 1 high pos
    public static double clawCpos1 = 1;
    // 0.4 is close
    // 0.9 is open
    public static double sweeperCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double wristCpos2 = 1;
    // 0.07 low pos
    // 0.45 give to ea
    // 0.5 high pos
    public static double clawCpos2 = 0.5;
    // 0.5 is close
    // 0.6 is grab pos
    // 1 is open pos
    public static double armCpos = 0.23;
    // 0.88 low pos
    // 0.72 grab from sa
    // 0 high pos
    public static double subArmCpos = 1;
    // 0 low pos
    // 0.45 high pos
    public static double rotationalCpos = 0.5;
    // 0 middle pos
    // 0.45 high pos
    // misc
    private static boolean ran = false;
    public static boolean redSide = true;
    public static int extendArmSpeed = 100;
    public static double wheelSpeed = 1;
    public static double rotationalSpeed = 0.2;
    private boolean preset = false;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static int slidesTARGET = 0;
    public static int eaLimitHigh = 2959;
    public static int eaLimitLow = -60;
    private static double power = 0;
    @Config("MainV4 Presets")
    public static class presets {
        public static CustomPresets humanPlayer = new CustomPresets(
                eaLimitLow,
                1.0,
                -1.0,
                0.0,
                -1.0,
                0.6,
                0.92,
                -1.0);
        public static CustomPresets highBasket = new CustomPresets(
                2500,
                -1.0,
                -1.0,
                0.4,
                -1.0,
                1.0,
                0.8,
                -1.0);
        public static CustomPresets lowBasket = new CustomPresets(
                1300,
                -1.0,
                -1.0,
                0.4,
                -1.0,
                1.0,
                0.8,
                -1.0);
        public static CustomPresets transition = new CustomPresets(
                eaLimitLow,
                1.0,
                1.0,
                0.0,
                0.9,
                0.5,
                0.18,
                0.52);
        public static CustomPresets specimen = new CustomPresets(
                1880,
                -1.0,
                -1.0,
                1.0,
                -1.0,
                0.45,
                0.25,
                -1.0);
    }
    @Override
    public void runOpMode()  {
        // hardware
        IMU imu = hardwareMap.get(IMU.class, "imu");
        MetroLib.setConstants(MConstants.class);
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PIDController controller = new PIDController(Math.sqrt(PIDTuneSlides.P), PIDTuneSlides.I, PIDTuneSlides.D);
        TouchSensor sensor = hardwareMap.get(TouchSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Limelight3A limelightMap = hardwareMap.get(Limelight3A.class, "limelight");
        // motors
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotorEx extendArm1 = hardwareMap.get(DcMotorEx.class, "ExtendArm1");
        DcMotorEx extendArm2 = hardwareMap.get(DcMotorEx.class, "ExtendArm2");
        // servos
        Servo sweeper = hardwareMap.get(Servo.class, "sweeper"); // 1x goBilda torque
        // ea
        Servo arm = hardwareMap.get(Servo.class, "arm"); // 2x axon
        Servo wrist1 = hardwareMap.get(Servo.class, "wrist1"); // 1x axon
        Servo claw1 = hardwareMap.get(Servo.class, "claw1"); // 1x axon
        // sa
        Servo submersibleArm1 = hardwareMap.get(Servo.class, "subArm"); // 1x axon
        Servo submersibleArm2 = hardwareMap.get(Servo.class, "subArm"); // 1x axon
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x 20kg
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        Servo rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0, 0.8);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0.4, 0.8);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        sweeper.setDirection(Servo.Direction.REVERSE);
        // breaks
        Motors.setBrakes(leftFront, rightFront, leftRear, rightRear);
        // reset encoders
        extendArm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // misc
        limelightMap.setPollRateHz(100); // update 100 times a second
        limelightMap.start();
        gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);
        gamepad2.setLedColor(255, 0, 255, Integer.MAX_VALUE);
        extendArm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        claw1.setPosition(clawCpos1);
        follower.startTeleopDrive();
        // calibration
        imu.resetYaw();
        Calibrate.TeleOp.calibrateStartup(List.of(extendArm1, extendArm2));
        follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        Calibrate.Auto.clearEverything();
        // telemetry
        telemetry.addData("RESETTING", "DONE!");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // variables
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                // misc
                // servos
                wrist1.setPosition(wristCpos1);
                wrist2.setPosition(wristCpos2);
                claw1.setPosition(clawCpos1);
                claw2.setPosition(clawCpos2);
                arm.setPosition(armCpos);
                submersibleArm1.setPosition(subArmCpos);
                sweeper.setPosition(sweeperCpos);
                rotation.setPosition(rotationalCpos);
                if (buttonClick(gamepad1.share) || buttonClick(gamepad2.share)) redSide = !redSide;
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
                    double rightFrontPower = (forward - strafe - turn) * wheelSpeed;
                    double rightBackPower = (forward + strafe - turn) * wheelSpeed;
                    // power
                    leftFront.setPower(leftFrontPower);
                    leftRear.setPower(leftBackPower);
                    rightFront.setPower(rightFrontPower);
                    rightRear.setPower(rightBackPower);
                } else {
                    follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);
                    follower.update();
                }
                // extendArm code
                controller.setPID(Math.sqrt(PIDTuneSlides.P), PIDTuneSlides.I, PIDTuneSlides.D);
                int eaCpos1 = extendArm1.getCurrentPosition();
                int eaCpos2 = extendArm2.getCurrentPosition();
                double ff = PIDTuneSlides.F;
                // controls
                if (gamepad2.dpad_up && eaCpos1 < eaLimitHigh) {
                    double pid = controller.calculate(eaCpos1, eaLimitHigh);
                    power = pid + ff;
                    extendArm1.setPower(power);
                    extendArm2.setPower(power);
                } else if (gamepad2.dpad_down && eaCpos1 > eaLimitLow) {
                    double pid = controller.calculate(eaCpos1, eaLimitLow);
                    power = pid + ff;
                    extendArm1.setPower(power);
                    extendArm2.setPower(power);
                } else {
                    extendArm1.setPower(ff);
                    extendArm2.setPower(ff);
                }
                if (sensor.isPressed()) {
                    extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extendArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extendArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                // submersibleArm code
                if (gamepad1.dpad_up) {
                    subArmCpos = 0.45;
                } else if (gamepad1.dpad_down) {
                    subArmCpos = 1;
                }
                // preset code
                /**
                 * GAMEPAD 1
                 *   X / ▢         - Grab sample using limelight
                 *   Y / Δ         - EMPTY
                 *   B / O         - Grab from Human Player Preset
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
                // humanPlayer pos
                if (gamepad1.b) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = presets.humanPlayer.extendArm != -1.0 ? (int) presets.humanPlayer.extendArm : eaCpos1;
                    subArmCpos = presets.humanPlayer.subArm != -1.0 ? presets.humanPlayer.subArm : subArmCpos;
                    clawCpos2 = presets.humanPlayer.claw2 != -1.0 ? presets.humanPlayer.claw2 : clawCpos2;
                    wristCpos2 = presets.humanPlayer.wrist2 != -1.0 ? presets.humanPlayer.wrist2 : wristCpos2;
                    wristCpos1 = presets.humanPlayer.wrist1 != -1.0 ? presets.humanPlayer.wrist1 : wristCpos1;
                    clawCpos1 = presets.humanPlayer.claw1 != -1.0 ? presets.humanPlayer.claw1 : clawCpos1;
                    armCpos = presets.humanPlayer.arm != -1.0 ? presets.humanPlayer.arm : armCpos;
                    rotationalCpos = presets.humanPlayer.rotational1 != -1.0 ? presets.humanPlayer.rotational1 : rotationalCpos;
                    preset = true;
                }
                // limelight grabbing
                if (gamepad1.x) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = 0;
                    subArmCpos = 1;
                    if (true) {
                        wristCpos2 = 0.1;
                        Timer.wait(300);
                        claw2.setPosition(0.55);
                        preset = true;
                    }
                }
                /**
                 * GAMEPAD 2
                 *   X / ▢         - Transition from Submersible arm to Extend arm
                 *   Y / Δ         - Place in High Basket
                 *   B / O         - Score Specimen Preset
                 *   A / X         - Place in Low basket
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
                    slidesTARGET = presets.highBasket.extendArm != -1.0 ? (int) presets.highBasket.extendArm : eaCpos1;
                    subArmCpos = presets.highBasket.subArm != -1.0 ? presets.highBasket.subArm : subArmCpos;
                    clawCpos2 = presets.highBasket.claw2 != -1.0 ? presets.highBasket.claw2 : clawCpos2;
                    wristCpos2 = presets.highBasket.wrist2 != -1.0 ? presets.highBasket.wrist2 : wristCpos2;
                    wristCpos1 = presets.highBasket.wrist1 != -1.0 ? presets.highBasket.wrist1 : wristCpos1;
                    clawCpos1 = presets.highBasket.claw1 != -1.0 ? presets.highBasket.claw1 : clawCpos1;
                    armCpos = presets.highBasket.arm != -1.0 ? presets.highBasket.arm : armCpos;
                    rotationalCpos = presets.highBasket.rotational1 != -1.0 ? presets.highBasket.rotational1 : rotationalCpos;
                    preset = true;
                }
                // low basket pos
                if (gamepad2.a) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = presets.lowBasket.extendArm != -1.0 ? (int) presets.lowBasket.extendArm : eaCpos1;
                    subArmCpos = presets.lowBasket.subArm != -1.0 ? presets.lowBasket.subArm : subArmCpos;
                    clawCpos2 = presets.lowBasket.claw2 != -1.0 ? presets.lowBasket.claw2 : clawCpos2;
                    wristCpos2 = presets.lowBasket.wrist2 != -1.0 ? presets.lowBasket.wrist2 : wristCpos2;
                    wristCpos1 = presets.lowBasket.wrist1 != -1.0 ? presets.lowBasket.wrist1 : wristCpos1;
                    clawCpos1 = presets.lowBasket.claw1 != -1.0 ? presets.lowBasket.claw1 : clawCpos1;
                    armCpos = presets.lowBasket.arm != -1.0 ? presets.lowBasket.arm : armCpos;
                    rotationalCpos = presets.lowBasket.rotational1 != -1.0 ? presets.lowBasket.rotational1 : rotationalCpos;
                    preset = true;
                }
                // transition pos
                if (gamepad2.x) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = presets.transition.extendArm != -1.0 ? (int) presets.transition.extendArm : eaCpos1;
                    subArmCpos = presets.transition.subArm != -1.0 ? presets.transition.subArm : subArmCpos;
                    clawCpos2 = presets.transition.claw2 != -1.0 ? presets.transition.claw2 : clawCpos2;
                    wristCpos2 = presets.transition.wrist2 != -1.0 ? presets.transition.wrist2 : wristCpos2;
                    wristCpos1 = presets.transition.wrist1 != -1.0 ? presets.transition.wrist1 : wristCpos1;
                    clawCpos1 = presets.transition.claw1 != -1.0 ? presets.transition.claw1 : clawCpos1;
                    armCpos = presets.transition.arm != -1.0 ? presets.transition.arm : armCpos;
                    rotationalCpos = presets.transition.rotational1 != -1.0 ? presets.transition.rotational1 : rotationalCpos;
                    preset = true;
                }
                // specimen pos
                if (gamepad2.b) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = presets.specimen.extendArm != -1.0 ? (int) presets.specimen.extendArm : eaCpos1;
                    subArmCpos = presets.specimen.subArm != -1.0 ? presets.specimen.subArm : subArmCpos;
                    clawCpos2 = presets.specimen.claw2 != -1.0 ? presets.specimen.claw2 : clawCpos2;
                    wristCpos2 = presets.specimen.wrist2 != -1.0 ? presets.specimen.wrist2 : wristCpos2;
                    wristCpos1 = presets.specimen.wrist1 != -1.0 ? presets.specimen.wrist1 : wristCpos1;
                    clawCpos1 = presets.specimen.claw1 != -1.0 ? presets.specimen.claw1 : clawCpos1;
                    armCpos = presets.specimen.arm != -1.0 ? presets.specimen.arm : armCpos;
                    rotationalCpos = presets.specimen.rotational1 != -1.0 ? presets.specimen.rotational1 : rotationalCpos;
                    preset = true;
                }
                if (preset) {
                    double pid = controller.calculate(eaCpos1, slidesTARGET);
                    power = pid + ff;
                    extendArm1.setPower(power);
                    extendArm2.setPower(power);
                    if (Math.abs(power) <= 0.15) {
                        preset = false;
                    }
                }
                // claws
                if (gamepad2.left_trigger > 0 || gamepad2.right_bumper) {
                    clawCpos1 = 0;
                } else if (gamepad2.right_trigger > 0 || gamepad2.left_bumper) {
                    clawCpos1 = 1;
                }
                if (gamepad1.left_bumper) {
                    clawCpos2 = 0;
                } else if (gamepad1.right_bumper) {
                    clawCpos2 = 1;
                }
                // wrist
                if (gamepad1.dpad_right) {
                    wristCpos2 = 0;
                } else if (gamepad1.dpad_left) {
                    wristCpos2 = 1;
                }
                if (gamepad2.dpad_left) {
                    wristCpos1 = 0;
                } else if(gamepad2.dpad_right) {
                    wristCpos1 = 0.6;
                }
                // arm
                if (gamepad2.right_stick_y > 0.3) {
                    armCpos = 0.8;
                    wristCpos1 = 1;
                } else if (gamepad2.right_stick_y < -0.3) {
                    armCpos = 0.92;
                    wristCpos1 = 0.6;
                }
                // rotate
                if (gamepad1.right_trigger > 0) {
                    rotationalCpos += rotationalSpeed;
                    if (rotationalCpos > 0.7) rotationalCpos = 0.7;
                } else if (gamepad1.left_trigger > 0) {
                    rotationalCpos -= rotationalSpeed;
                    if (rotationalCpos < 0.2) rotationalCpos = 0.2;
                }
                // color sensor code
                if (Sensor.isRedGrabbed()) {
                    gamepad1.setLedColor(255, 0, 0, Integer.MAX_VALUE);
                    gamepad2.setLedColor(255, 0, 0, Integer.MAX_VALUE);
                    if (!ran) {
                        gamepad1.rumble(20, 20, 1000);
                        gamepad1.rumble(20, 20, 1000);
                        ran = true;
                    }
                } else if (Sensor.isBlueGrabbed()) {
                    gamepad1.setLedColor(0, 0, 255, Integer.MAX_VALUE);
                    gamepad2.setLedColor(0, 0, 255, Integer.MAX_VALUE);
                    if (!ran) {
                        gamepad1.rumble(20, 20, 1000);
                        gamepad1.rumble(20, 20, 1000);
                        ran = true;
                    }
                } else if (Sensor.isYellowGrabbed()) {
                    gamepad1.setLedColor(255, 255, 0, Integer.MAX_VALUE);
                    gamepad2.setLedColor(255, 255, 0, Integer.MAX_VALUE);
                    if (!ran) {
                        gamepad1.rumble(20, 20, 1000);
                        gamepad1.rumble(20, 20, 1000);
                        ran = true;
                    }
                } else {
                    gamepad1.setLedColor(0, 255, 0, Integer.MAX_VALUE);
                    gamepad2.setLedColor(255, 0, 255, Integer.MAX_VALUE);
                    ran = false;
                }
                // alerts
                // 15 seconds left and start of match
                if (Timer.TeleOp.alert(15) || Timer.TeleOp.alert(150)) {
                    gamepad1.rumble(20, 20, 1000);
                    gamepad1.rumble(20, 20, 1000);
                }
                // telemetry
                Calibrate.TeleOp.getStartPositions();
                telemetry.addData("PIDF", "P: " + PIDTuneSlides.P + " I: " + PIDTuneSlides.I + " D: " + PIDTuneSlides.D + " F: " + PIDTuneSlides.F);
                telemetry.addData("target", slidesTARGET);
                telemetry.addData("eaCpos1", eaCpos1);
                telemetry.addData("eaCpos2", eaCpos2);
                telemetry.addData("eaPower", power);
                telemetry.addData("error1", Math.abs(slidesTARGET - eaCpos1));
                telemetry.addData("error2", Math.abs(slidesTARGET - eaCpos2));
                telemetry.addData("errorAvg", (Math.abs(slidesTARGET - eaCpos1) + Math.abs(slidesTARGET - eaCpos2)) / 2);
                telemetry.addData("DEBUG:", "PickUp " + (Sensor.pickUpRed() ? "RED" : Sensor.pickUpBlue() ? "BLUE" : Sensor.pickUpYellow() ? "YELLOW" : "NONE"));
                telemetry.addData("DEBUG:", "Grabbed " + (Sensor.isRedGrabbed() ? "RED" : Sensor.isBlueGrabbed() ? "BLUE" : Sensor.isYellowGrabbed() ? "YELLOW" : "NONE"));
                telemetry.addData("touch sensor", sensor.isPressed());
                telemetry.addData("touch sensor", sensor.getValue());
                telemetry.addData("Submersible Arm Position1:", submersibleArm1.getPosition());
                telemetry.addData("Wrist Position1:", wrist1.getPosition());
                telemetry.addData("Wrist Position2:", wrist2.getPosition());
                telemetry.addData("Claw Position1:", claw1.getPosition());
                telemetry.addData("Claw Position2:", claw2.getPosition());
                telemetry.addData("Arm Position:", arm.getPosition());
                telemetry.addData("Sweeper Position:", sweeper.getPosition());
                telemetry.addData("Rotation Position:", rotation.getPosition());
                telemetry.addData("triggersR?", gamepad1.right_trigger);
                telemetry.addData("triggersL?", gamepad1.left_trigger);
                telemetry.addData("Manually moving robot?", moving);
                telemetry.addData("Red side?", redSide);
                telemetry.update();
            }
        }
        if (isStopRequested()) {
            // stop code
        }
    }
    private boolean buttonPreviousState = false;
    public boolean buttonClick (boolean button) {
        boolean returnVal;
        returnVal = button && !buttonPreviousState;
        buttonPreviousState = button;
        return returnVal;
    }
}