/***
 * MAIN V6
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * coding from scratch because i hate my old code again
 * based off of MainV5 but better
 * started recoding at 5/12/25  @  11:38 am
 * finished recoding at 5/17/25 @ 11:32 am
 * robot v6 finished building at 5/31/25
***/
package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.CPR;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.D;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.F;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.I;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.INCHES_PER_REV;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.K;
import static org.firstinspires.ftc.teamcode.testCode.slides.PIDTuneSlides.P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LimelightState;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.utils.LynxUtils;
import org.firstinspires.ftc.teamcode.utils.TelemetryM;
import org.firstinspires.ftc.teamcode.variables.enums.ModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.PresetStates;
import org.firstinspires.ftc.teamcode.variables.enums.RotationStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.BasketsModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.SpecModeStates;
import org.firstinspires.ftc.teamcode.variables.enums.subEnums.SubModeStates;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import xyz.nin1275.MetroLib;
import xyz.nin1275.controllers.PID;
import xyz.nin1275.enums.SlidersStates;
import xyz.nin1275.subsystems.SlidesSS;
import xyz.nin1275.utils.Calibrate;
import xyz.nin1275.utils.CombinedServo;
import xyz.nin1275.utils.Motors;

@Configurable
@Config("MainV6")
@TeleOp(name="Main v6", group=".ftc23403")
public class MainV6_RECODE extends LinearOpMode {
    /**
     * @TODO driver practice bro
     * @TODO fix all the movements using state systems for everything
     * @TODO get all the new modes working and stuff
     * @TODO fix all the mode systems
     * MAIN V6 BY DAVID
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
    **/
    // servos
    public static double wristCpos1 = 1;
    public static double clawCpos1 = 1;
    public static double wristCpos2 = 1;
    public static double clawCpos2 = 1;
    public static double armCpos = 0.15;
    public static double subArmCpos = 1;
    public static double rotationalCpos2 = 0;
    public static double rotationalCpos1 = 0;
    // misc
    public static double wheelSpeedMinEA = 0.7;
    public static double wheelSpeedMinSA = 0.8;
    private double wheelSpeed = wheelSpeedMax;
    // power draw
    ElapsedTime loopTime;
    public static boolean bulkRead = true;
    private static boolean brOFF = false;
    private static boolean brON = false;
    // odometry
    public static boolean odoDrive = true;
    // extend arm
    public static double slidesTARGET = 0;
    public static double eaLimitHigh = 33.6;
    public static double eaLimitLow = 0;
    public static boolean eaCorrection = true;
    public static SlidesSS extendArmSS;
    // states
    SlidersStates extendArmState = SlidersStates.FLOATING;
    private static PresetStates presetState = PresetStates.NO_PRESET;
    RotationStates rotationState = RotationStates.MIDDLE;
    LimelightState llState = LimelightState.INIT;
    private static ModeStates mode = ModeStates.SPECIMEN;
    private static SubModeStates subStates = SubModeStates.RETURN;
    private static SpecModeStates specStates = SpecModeStates.GRAB;
    private static BasketsModeStates basketsStates = BasketsModeStates.RETURN;
    // config stuff
    public static double SUB_THROW_POS = 188;
    public static double EA_MAX_SPEED_DOWN = -0.4;
    public static boolean redSide = true;
    public static boolean debugMode = false;
    public static double wheelSpeedMax = 1;
    @Override
    public void runOpMode() {
        // hardware
        Follower follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        PID controller = new PID(Math.sqrt(P), I, D);
        MetroLib.teleOp.init(this, telemetry, gamepad1, gamepad2, follower);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TelemetryM telemetryM = new TelemetryM(telemetry, debugMode);
        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.setPollRateHz(50);
        Vision.Limelight limelight = new Vision.Limelight(limelight3A, llState, follower);
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        LynxUtils.initLynx(hardwareMap);
        AnalogInput subArms = hardwareMap.get(AnalogInput.class, "subArms");
        AnalogInput arms = hardwareMap.get(AnalogInput.class, "arms");
        // gamepads
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        // motors
        CachingDcMotorEx leftFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        CachingDcMotorEx leftRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftRear"));
        CachingDcMotorEx rightFront = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));
        CachingDcMotorEx rightRear = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightRear"));
        CachingDcMotorEx extendArm1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm1"));
        CachingDcMotorEx extendArm2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "ExtendArm2"));
        // servos
        // ea
        CachingServo arm1 = new CachingServo(hardwareMap.get(Servo.class, "arm1")); // 1x axon max
        CachingServo arm2 = new CachingServo(hardwareMap.get(Servo.class, "arm2")); // 1x axon max
        CachingServo wrist1 = new CachingServo(hardwareMap.get(Servo.class, "wrist1")); // 1x axon mini
        CachingServo claw1 = new CachingServo(hardwareMap.get(Servo.class, "claw1")); // 1x axon mini
        CachingServo rotation1 = new CachingServo(hardwareMap.get(Servo.class, "rotation2")); // 1x axon max
        CombinedServo arm = new CombinedServo(arm1, arm2); // 2x axon max
        // sa
        CachingServo submersibleArm1 = new CachingServo(hardwareMap.get(Servo.class, "subArm1")); // 1x axon max
        CachingServo submersibleArm2 = new CachingServo(hardwareMap.get(Servo.class, "subArm2")); // 1x 25kg
        CachingServo wrist2 = new CachingServo(hardwareMap.get(Servo.class, "wrist2")); // 1x axon mini
        CachingServo claw2 = new CachingServo(hardwareMap.get(Servo.class, "claw2")); // 1x axon mini
        CachingServo rotation2 = new CachingServo(hardwareMap.get(Servo.class, "rotation1")); // 1x axon max
        CombinedServo subArm = new CombinedServo(submersibleArm1, submersibleArm2); // 1x axon max : 1x 25kg
        // limits
        claw2.scaleRange(0.01, 0.08);
        wrist2.scaleRange(0.05, 0.8);
        rotation2.scaleRange(0.43, 0.55);
        arm.scaleRange(0.12, 1);
        wrist1.scaleRange(0, 0.6);
        claw1.scaleRange(0, 0.4);
        subArm.scaleRange(0.385, 0.85);
        // reverse
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        extendArm2.setDirection(DcMotorEx.Direction.REVERSE);
        // breaks
        Motors.setBrakes(leftFront, rightFront, leftRear, rightRear);
        // colors
        gamepad1.setLedColor(0, 255, 255, -1);
        gamepad2.setLedColor(0, 255, 0, -1);
        LynxUtils.setLynxColor(255, 0, 255);
        // starting pos
        wrist1.setPosition(wristCpos1 = 1);
        wrist2.setPosition(wristCpos2 = 1);
        claw1.setPosition(clawCpos1 = 1);
        claw2.setPosition(clawCpos2 = 1);
        arm.setPosition(armCpos = 0.15);
        subArm.setPosition(subArmCpos = 1);
        rotation1.setPosition(rotationalCpos1 = 0);
        rotation2.setPosition(rotationalCpos2 = 0);
        // calibration
        if (bulkRead) {
            brON = true;
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        hardwareMap.get(IMU.class, "imu").resetYaw();
        if (Calibrate.Auto.getLastKnownPos() != null) follower.setStartingPose(Calibrate.Auto.getLastKnownPos());
        else follower.setStartingPose(new Pose(9,63.4,0));
        Calibrate.Auto.clearEverything();
        // Draw the robot on the dashboard
        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        if (Calibrate.Auto.getLastKnownPos() != null) poseUpdater.setStartingPose(Calibrate.Auto.getLastKnownPos());
        else poseUpdater.setStartingPose(new Pose(9,63.4,0));
        DashboardPoseTracker dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
        // setup slides
        extendArmSS = new SlidesSS(extendArm1, extendArm2, controller, K, F, CPR, INCHES_PER_REV, MainV6.eaLimitHigh, MainV6.eaLimitLow, eaCorrection, false);
        // misc
        loopTime = new ElapsedTime();
        loopTime.reset();
        // telemetry
        telemetryM.addLine("BEASTKIT Team 23403!");
        telemetryM.addLine(true, "INIT DONE!");
        telemetryM.update();
        waitForStart();
        if (opModeIsActive()) {
            follower.startTeleopDrive();
            while (opModeIsActive()) {
                // variables
                limelight.update();
                llState = limelight.getState();
                extendArmState = extendArmSS.getState();
                extendArmSS.setEaCorrection(eaCorrection);
                extendArmSS.setLimits(MainV6.eaLimitHigh, MainV6.eaLimitLow);
                extendArmSS.setMaxSpeedDown(EA_MAX_SPEED_DOWN);
                limelight.setFollower(follower);
                telemetryM.setDebug(debugMode);
                boolean moving = Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0;
                // analog
                double subArmPos = subArms.getVoltage() / 3.3 * 360;
                double armPos = arms.getVoltage() / 3.3 * 360;
                // gamepad stuff
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);
                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
                // servos
                wrist1.setPosition(wristCpos1);
                wrist2.setPosition(wristCpos2);
                claw1.setPosition(clawCpos1);
                claw2.setPosition(clawCpos2);
                arm.setPosition(armCpos);
                subArm.setPosition(subArmCpos);
                rotation1.setPosition(rotationalCpos1);
                rotation2.setPosition(rotationalCpos2);
                // bulk read
                for (LynxModule hub : allHubs) {
                    if (bulkRead) {
                        if (!brON) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
                        hub.clearBulkCache();
                        brON = true;
                        brOFF = false;
                    } else {
                        if (!brOFF) hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                        brOFF = true;
                        brON = false;
                    }
                }
                // field side
                if ((currentGamepad1.share && !previousGamepad1.share) || (currentGamepad2.share && !previousGamepad2.share)) redSide = !redSide;
                // toggle debug
                if ((currentGamepad1.options && !previousGamepad1.options) || (currentGamepad2.options && !previousGamepad2.options)) debugMode = !debugMode;
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
                    follower.setMaxPower(wheelSpeed);
                    follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
                    follower.update();
                }
                if (!moving && !odoDrive) {
                    leftFront.setPower(0);
                    leftRear.setPower(0);
                    rightFront.setPower(0);
                    rightRear.setPower(0);
                }
                // claw
                if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) clawCpos1 = 0;
                else if (!(currentGamepad2.right_trigger > 0) && previousGamepad2.left_trigger > 0) clawCpos1 = 1;
                // extendArm code
                extendArmSS.update(gamepad2.right_stick_y > 0.5, gamepad2.right_stick_y < 0.5);
                // submersibleArm code
                if (gamepad1.right_stick_y > 0.8) {
                    subArmCpos = 0;
                } else if (gamepad1.right_stick_y < 0.8) {
                    subArmCpos = 1;
                }
            }
        }
    }
}
