/***
 * MAIN V5
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch because i hate my old code
 * based off of MainV4 but better and advancer
 * started recoding at 4/4/25  @  7:55 pm
 * robot v5 finished building at 4/7/25
 */
package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightState;
import org.firstinspires.ftc.teamcode.testCode.PIDTuneSlides;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;
import org.firstinspires.ftc.teamcode.variables.enums.extendArmStates;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.Motors;
import xyz.nin1275.utils.Sensor;
import xyz.nin1275.utils.Timer;

@Config("MainV5")
@TeleOp(name="Main v5", group=".ftc23403")
public class MainV5 extends LinearOpMode {
    /**
     * @TODO have odometry driving working
     * @TODO finish all the presets PROPERLY FOR ONCE GODDAMNIT
     * @TODO get limelight working in here
     * @TODO add color sensor shit
     * @TODO get all the finite state machines working in here gang
     * MAIN V5 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
    */
    // servos
    public static double wristCpos1 = 0;
    public static double clawCpos1 = 1;
    public static double sweeperCpos = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 0.5;
    public static double armCpos = 0.23;
    public static double subArmCpos = 1;
    public static double rotationalCpos = 0.5;
    // misc
    public static boolean redSide = true;
    public static int extendArmSpeed = 100;
    public static double wheelSpeed = 1;
    public static double rotationalSpeed = 0.2;
    // odometry
    public static boolean odoDrive = false;
    // extend arm
    public static int slidesTARGET = 0;
    public static int eaLimitHigh = 2959;
    public static int eaLimitLow = -60;
    private static double power = 0;
    public static boolean eaCorrection = true;
    private static extendArmStates extendArmState = extendArmStates.FLOATING;
    // presets
    @Config("MainV5 Presets")
    public static class presets {
        public static double preScoreArmPos = 0.3;
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
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PIDController controller = new PIDController(Math.sqrt(PIDTuneSlides.P), PIDTuneSlides.I, PIDTuneSlides.D);
        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "sensor");
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower, sensor);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
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
        Servo submersibleArm1 = hardwareMap.get(Servo.class, "subArm1"); // 1x axon
        Servo submersibleArm2 = hardwareMap.get(Servo.class, "subArm2"); // 1x axon
        Servo wrist2 = hardwareMap.get(Servo.class, "wrist2"); // 1x 20kg
        Servo claw2 = hardwareMap.get(Servo.class, "claw2"); // 1x goBilda speed
        Servo rotation = hardwareMap.get(Servo.class, "rotation"); // 1x goBilda speed
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0, 0.8);
        rotation.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0.2, 1);
        claw1.scaleRange(0.4, 0.8);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        sweeper.setDirection(Servo.Direction.REVERSE);
        // breaks
        Motors.setBrakes(List.of(leftFront, rightFront, leftRear, rightRear));
        // reset encoders
        Motors.resetEncoders(List.of(extendArm1, extendArm2));
        // misc
        Limelight limelight = new Limelight(limelight3A, wrist2, submersibleArm1, telemetry);
        gamepad1.setLedColor(0, 255, 0, -1);
        gamepad2.setLedColor(255, 0, 255, -1);
        extendArm1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extendArm2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        claw1.setPosition(clawCpos1);
        // calibration
        hardwareMap.get(IMU.class, "imu").resetYaw();
        if (Calibrate.Auto.getLastKnownPos() != null) {
            follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        } else {
            follower.setStartingPose(new Pose(0,0,0));
        }
        Calibrate.Auto.clearEverything();
        extendArmState = extendArmStates.RESETTING_ZERO_POS;
        // telemetry
        telemetry.addData("INIT", "DONE!");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                boolean moving = gamepad1.left_stick_x > 0 || gamepad1.left_stick_x < 0 || gamepad1.left_stick_y > 0 || gamepad1.left_stick_y < 0 || gamepad1.right_stick_x > 0 || gamepad1.right_stick_x < 0;
                // servos
                wrist1.setPosition(wristCpos1);
                wrist2.setPosition(wristCpos2);
                claw1.setPosition(clawCpos1);
                claw2.setPosition(clawCpos2);
                arm.setPosition(armCpos);
                submersibleArm1.setPosition(subArmCpos);
                sweeper.setPosition(sweeperCpos);
                rotation.setPosition(rotationalCpos);
                // field side
                if (gamepad1.share || gamepad2.share) {
                    redSide = !redSide;
                    Timer.wait(500);
                }
                // movements
                if (!odoDrive) {
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
                    power = pid + (eaCorrection ? ff : 0);
                    extendArm1.setPower(power);
                    extendArm2.setPower(power);
                    extendArmState = extendArmStates.MANUAL_MOVEMENT;
                } else if (gamepad2.dpad_down && eaCpos1 > eaLimitLow) {
                    double pid = controller.calculate(eaCpos1, eaLimitLow);
                    power = pid + (eaCorrection ? ff : 0);
                    extendArm1.setPower(power);
                    extendArm2.setPower(power);
                    extendArmState = extendArmStates.MANUAL_MOVEMENT;
                } else if (Math.abs(eaCpos1 - eaLimitLow) > 60) {
                    extendArm1.setPower(eaCorrection ? ff : 0);
                    extendArm2.setPower(eaCorrection ? ff : 0);
                    if (extendArmState == extendArmStates.PRESET_REACHED) Timer.wait(500);
                    extendArmState = eaCorrection ? extendArmStates.FORCE_FEED_BACK : extendArmStates.FLOATING;
                }
                // states
                if (Math.abs(eaCpos1 - eaLimitHigh) > 60) {
                    extendArmState = extendArmStates.MAX_POS;
                } else if (Math.abs(eaCpos1 - eaLimitLow) > 60) {
                    extendArmState = extendArmStates.LOW_POS;
                }
                // preset controls
                if (extendArmState == extendArmStates.MOVING_TO_PRESET) {
                    double pid = controller.calculate(eaCpos1, slidesTARGET);
                    power = pid + (eaCorrection ? ff : 0);
                    extendArm1.setPower(power);
                    extendArm2.setPower(power);
                    // check if we are at the target by 50 encoders
                    if (Math.abs(eaCpos1 - slidesTARGET) < 50) {
                        extendArmState = extendArmStates.PRESET_REACHED;
                    }
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
                    slidesTARGET = MainV5.presets.humanPlayer.extendArm != -1.0 ? MainV5.presets.humanPlayer.extendArm : eaCpos1;
                    subArmCpos = MainV5.presets.humanPlayer.subArm != -1.0 ? MainV5.presets.humanPlayer.subArm : subArmCpos;
                    clawCpos2 = MainV5.presets.humanPlayer.claw2 != -1.0 ? MainV5.presets.humanPlayer.claw2 : clawCpos2;
                    wristCpos2 = MainV5.presets.humanPlayer.wrist2 != -1.0 ? MainV5.presets.humanPlayer.wrist2 : wristCpos2;
                    wristCpos1 = MainV5.presets.humanPlayer.wrist1 != -1.0 ? MainV5.presets.humanPlayer.wrist1 : wristCpos1;
                    clawCpos1 = MainV5.presets.humanPlayer.claw1 != -1.0 ? MainV5.presets.humanPlayer.claw1 : clawCpos1;
                    armCpos = MainV5.presets.humanPlayer.arm != -1.0 ? MainV5.presets.humanPlayer.arm : armCpos;
                    rotationalCpos = MainV5.presets.humanPlayer.rotational != -1.0 ? MainV5.presets.humanPlayer.rotational : rotationalCpos;
                    extendArmState = extendArmStates.MOVING_TO_PRESET;
                }
                // limelight grabbing
                if (gamepad1.x) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = 0;
                    subArmCpos = 1;
                    if (limelight.search() == LimelightState.SAMPLE_REACHED) {
                        wristCpos2 = 0.1;
                        Timer.wait(300);
                        claw2.setPosition(0.55);
                        extendArmState = extendArmStates.MOVING_TO_PRESET;
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
                    slidesTARGET = MainV5.presets.highBasket.extendArm != -1.0 ? MainV5.presets.highBasket.extendArm : eaCpos1;
                    subArmCpos = MainV5.presets.highBasket.subArm != -1.0 ? MainV5.presets.highBasket.subArm : subArmCpos;
                    clawCpos2 = MainV5.presets.highBasket.claw2 != -1.0 ? MainV5.presets.highBasket.claw2 : clawCpos2;
                    wristCpos2 = MainV5.presets.highBasket.wrist2 != -1.0 ? MainV5.presets.highBasket.wrist2 : wristCpos2;
                    wristCpos1 = MainV5.presets.highBasket.wrist1 != -1.0 ? MainV5.presets.highBasket.wrist1 : wristCpos1;
                    clawCpos1 = MainV5.presets.highBasket.claw1 != -1.0 ? MainV5.presets.highBasket.claw1 : clawCpos1;
                    armCpos = MainV5.presets.highBasket.arm != -1.0 ? MainV5.presets.highBasket.arm : armCpos;
                    rotationalCpos = MainV5.presets.highBasket.rotational != -1.0 ? MainV5.presets.highBasket.rotational : rotationalCpos;
                    extendArmState = extendArmStates.MOVING_TO_PRESET;
                }
                // low basket pos
                if (gamepad2.a) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = MainV5.presets.lowBasket.extendArm != -1.0 ? MainV5.presets.lowBasket.extendArm : eaCpos1;
                    subArmCpos = MainV5.presets.lowBasket.subArm != -1.0 ? MainV5.presets.lowBasket.subArm : subArmCpos;
                    clawCpos2 = MainV5.presets.lowBasket.claw2 != -1.0 ? MainV5.presets.lowBasket.claw2 : clawCpos2;
                    wristCpos2 = MainV5.presets.lowBasket.wrist2 != -1.0 ? MainV5.presets.lowBasket.wrist2 : wristCpos2;
                    wristCpos1 = MainV5.presets.lowBasket.wrist1 != -1.0 ? MainV5.presets.lowBasket.wrist1 : wristCpos1;
                    clawCpos1 = MainV5.presets.lowBasket.claw1 != -1.0 ? MainV5.presets.lowBasket.claw1 : clawCpos1;
                    armCpos = MainV5.presets.lowBasket.arm != -1.0 ? MainV5.presets.lowBasket.arm : armCpos;
                    rotationalCpos = MainV5.presets.lowBasket.rotational != -1.0 ? MainV5.presets.lowBasket.rotational : rotationalCpos;
                    extendArmState = extendArmStates.MOVING_TO_PRESET;
                }
                // transition pos
                if (gamepad2.x) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = MainV5.presets.transition.extendArm != -1.0 ? MainV5.presets.transition.extendArm : eaCpos1;
                    subArmCpos = MainV5.presets.transition.subArm != -1.0 ? MainV5.presets.transition.subArm : subArmCpos;
                    clawCpos2 = MainV5.presets.transition.claw2 != -1.0 ? MainV5.presets.transition.claw2 : clawCpos2;
                    wristCpos2 = MainV5.presets.transition.wrist2 != -1.0 ? MainV5.presets.transition.wrist2 : wristCpos2;
                    wristCpos1 = MainV5.presets.transition.wrist1 != -1.0 ? MainV5.presets.transition.wrist1 : wristCpos1;
                    clawCpos1 = MainV5.presets.transition.claw1 != -1.0 ? MainV5.presets.transition.claw1 : clawCpos1;
                    armCpos = MainV5.presets.transition.arm != -1.0 ? MainV5.presets.transition.arm : armCpos;
                    rotationalCpos = MainV5.presets.transition.rotational != -1.0 ? MainV5.presets.transition.rotational : rotationalCpos;
                    extendArmState = extendArmStates.MOVING_TO_PRESET;
                }
                // specimen pos
                if (gamepad2.b) {
                    // use correction code cuz its easier fr fr
                    slidesTARGET = MainV5.presets.specimen.extendArm != -1.0 ? MainV5.presets.specimen.extendArm : eaCpos1;
                    subArmCpos = MainV5.presets.specimen.subArm != -1.0 ? MainV5.presets.specimen.subArm : subArmCpos;
                    clawCpos2 = MainV5.presets.specimen.claw2 != -1.0 ? MainV5.presets.specimen.claw2 : clawCpos2;
                    wristCpos2 = MainV5.presets.specimen.wrist2 != -1.0 ? MainV5.presets.specimen.wrist2 : wristCpos2;
                    wristCpos1 = MainV5.presets.specimen.wrist1 != -1.0 ? MainV5.presets.specimen.wrist1 : wristCpos1;
                    clawCpos1 = MainV5.presets.specimen.claw1 != -1.0 ? MainV5.presets.specimen.claw1 : clawCpos1;
                    armCpos = MainV5.presets.specimen.arm != -1.0 ? MainV5.presets.specimen.arm : armCpos;
                    rotationalCpos = MainV5.presets.specimen.rotational != -1.0 ? MainV5.presets.specimen.rotational : rotationalCpos;
                    extendArmState = extendArmStates.MOVING_TO_PRESET;
                }
                // auto move arm to score when we pickup from human player
                if (armCpos == presets.humanPlayer.arm && wristCpos1 == presets.humanPlayer.wrist1 && clawCpos1 == 1) {
                    Timer.wait(200);
                    if (moving && clawCpos1 == 1) {
                        armCpos = presets.preScoreArmPos;
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
                // telemetry
                telemetry.addData("extendArmState", extendArmState);
                telemetry.addData("PIDF", "P: " + PIDTuneSlides.P + " I: " + PIDTuneSlides.I + " D: " + PIDTuneSlides.D + " F: " + PIDTuneSlides.F);
                telemetry.addData("target", slidesTARGET);
                telemetry.addData("eaCpos1", eaCpos1);
                telemetry.addData("eaCpos2", eaCpos2);
                telemetry.addData("eaPower", power);
                telemetry.addData("preset error1", Math.abs(slidesTARGET - eaCpos1));
                telemetry.addData("preset error2", Math.abs(slidesTARGET - eaCpos2));
                telemetry.addData("preset errorAvg", (Math.abs(slidesTARGET - eaCpos1) + Math.abs(slidesTARGET - eaCpos2)) / 2);
                telemetry.addData("DEBUG:", "PickUp " + (Sensor.pickUpRed() ? "RED" : Sensor.pickUpBlue() ? "BLUE" : Sensor.pickUpYellow() ? "YELLOW" : "NONE"));
                telemetry.addData("DEBUG:", "Grabbed " + (Sensor.isRedGrabbed() ? "RED" : Sensor.isBlueGrabbed() ? "BLUE" : Sensor.isYellowGrabbed() ? "YELLOW" : "NONE"));
                telemetry.addData("Sensor Distance MM:", sensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Sensor RGBA:", "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue() + " A: " + sensor.alpha());
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
                telemetry.addData("Red side?", redSide);
                telemetry.update();
            }
        }
        if (isStopRequested()) {
            // stop code
            // turn off servos on the servo hub or spm if we get them
        }
    }
}
