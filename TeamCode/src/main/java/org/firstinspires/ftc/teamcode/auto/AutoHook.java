package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * MetroBotics/Code Conductors auto using odometry.
 * Started code  @  2/14/25  @  5:37 pm
 * Expected to finish code  @  2/26/25
 * It is a 1+3 specimen auto. It hangs a preloaded specimen and then hang another specimen then push the 3 samples from the ground and either park or hang them depending on what time we have.
 * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
 * @version 2.0, 2/15/25
 */

@Config("Auto Hook")
@Autonomous (name = "Auto Hook", group = ".ftc23403")
public class AutoHook extends OpMode {
    private Telemetry telemetryA;
    private String state;
    private Follower follower;
    private PathChain specimen;
    private static double speed;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        // reset imu
        hardwareMap.get(IMU.class, "imu").resetYaw();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        // path
        specimen = follower.pathBuilder()
                .addPath(
                // Line 1
                new BezierCurve(
                        new Point(9.757, 84.983, Point.CARTESIAN),
                        new Point(37.352, 115.495, Point.CARTESIAN),
                        new Point(36.668, 84.983, Point.CARTESIAN)
                )
        )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(36.668, 84.983, Point.CARTESIAN),
                                new Point(94.000, 64.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(94.000, 64.000, Point.CARTESIAN),
                                new Point(35.000, 98.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(35.000, 98.000, Point.CARTESIAN),
                                new Point(142.000, 118.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(142.000, 118.000, Point.CARTESIAN),
                                new Point(7.000, 100.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(7.000, 100.000, Point.CARTESIAN),
                                new Point(108.000, 7.000, Point.CARTESIAN),
                                new Point(113.000, 104.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(113.000, 104.000, Point.CARTESIAN),
                                new Point(17.000, 140.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(17.000, 140.000, Point.CARTESIAN),
                                new Point(86.000, 113.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(86.000, 113.000, Point.CARTESIAN),
                                new Point(37.000, 137.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(37.000, 137.000, Point.CARTESIAN),
                                new Point(5.000, 134.000, Point.CARTESIAN),
                                new Point(4.000, 51.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(4.000, 51.000, Point.CARTESIAN),
                                new Point(25.000, 132.000, Point.CARTESIAN),
                                new Point(89.000, 86.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(89.000, 86.000, Point.CARTESIAN),
                                new Point(5.000, 111.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        // telemetry
        follower.followPath(specimen);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("debug");
        telemetryA.update();
    }

    /**
     * Setup arm movements and slider movements and turn stuff here
     * custom made to work with getState() function.
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    public void pathState() {
        switch (state) {
            case "Running1":
                // code
                extendArmMove(100, 100, 0.7);
                break;
            case "Running2":
                extendArmMove(-100, -100, 0.7);
                pause(1000);
                break;
            case "Running3":
                // rest of movements
                break;
            case "Running4":

                break;
            case "Running5":

                break;
            case "Running6":

                break;
            case "Running7":

                break;
        }
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (follower.atParametricEnd()) {
            follower.followPath(specimen);
        }
        follower.telemetryDebug(telemetryA);
        follower.setMaxPower(speed);
        telemetry.addData("state", state);
        telemetry.update();
    }

    /**
     * Encoder movements for the arms and sliders,
     * also for the robot's rotation cuz ion wanna use odometry rotation.
     * Encoder movement was specially made to work even if only one motor on the drive train has encoders,
     * meaning all movements will fully work as long as it meets the minimum requirements of,
     * at least 1 motor on the drive train with an encoder.
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    private void extendArmMove(int dis1, int dis2, double power) {
        DcMotor extendArm1 = hardwareMap.dcMotor.get("ExtendArm1");
        DcMotor extendArm2 = hardwareMap.dcMotor.get("ExtendArm2");
        // stuff
        extendArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        extendArm2.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        extendArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        extendArm1.setTargetPosition(dis1);
        extendArm2.setTargetPosition(dis2);
        // move moters
        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        extendArm1.setPower(power);
        extendArm2.setPower(power);
        int eaCpos1 = 0;
        int eaCpos2 = 0;
        while (extendArm1.isBusy() || extendArm2.isBusy()) {
            telemetry.addData("ExtendArm1Pos:", extendArm1.getCurrentPosition());
            telemetry.addData("ExtendArm2Pos:", extendArm2.getCurrentPosition());
            telemetry.update();
            eaCpos1 = extendArm1.getCurrentPosition();
            eaCpos2 = extendArm2.getCurrentPosition();
        }
        // stop
        extendArm1.setTargetPosition(eaCpos1);
        extendArm2.setTargetPosition(eaCpos2);
        extendArm1.setPower(1);
        extendArm2.setPower(1);
        extendArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void submersibleArmMove(int dis1, int dis2, double power) {
        DcMotor submersibleArm1 = hardwareMap.dcMotor.get("SubArm1");
        DcMotor submersibleArm2 = hardwareMap.dcMotor.get("SubArm2");
        // stuff
        submersibleArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        submersibleArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // formula
        submersibleArm2.setDirection(DcMotor.Direction.REVERSE);
        // reset pos
        submersibleArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        submersibleArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        submersibleArm1.setTargetPosition(dis1);
        submersibleArm2.setTargetPosition(dis2);
        // move moters
        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        submersibleArm1.setPower(power);
        submersibleArm2.setPower(power);
        int saCpos1 = 0;
        int saCpos2 = 0;
        while (submersibleArm1.isBusy() || submersibleArm2.isBusy()) {
            telemetry.addData("SubmersibleArm1Pos:", submersibleArm1.getCurrentPosition());
            telemetry.addData("SubmersibleArm2Pos:", submersibleArm2.getCurrentPosition());
            telemetry.update();
            saCpos1 = submersibleArm1.getCurrentPosition();
            saCpos2 = submersibleArm2.getCurrentPosition();
        }
        // stop
        submersibleArm1.setTargetPosition(saCpos1);
        submersibleArm2.setTargetPosition(saCpos2);
        submersibleArm1.setPower(1);
        submersibleArm2.setPower(1);
        submersibleArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        submersibleArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void turn(int dis, double power) {
        // hardware
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("rightRear");
        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("leftFront");
        // formula
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        // reset pos
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // target
        leftBackDrive.setTargetPosition(dis);
        leftFrontDrive.setTargetPosition(dis);
        rightBackDrive.setTargetPosition(dis);
        rightFrontDrive.setTargetPosition(dis);
        // move moters
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // power
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);

        while (leftBackDrive.isBusy() && leftFrontDrive.isBusy() && rightBackDrive.isBusy()  && rightFrontDrive.isBusy() ) {
            telemetry.addData("LeftBackPos:", leftBackDrive.getCurrentPosition());
            telemetry.addData("LeftFrontPos:", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightBackPos:", rightBackDrive.getCurrentPosition());
            telemetry.addData("RightFrontPos:", rightFrontDrive.getCurrentPosition());
            telemetry.update();
        }
        // stop
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }
    private void claw(double pos) {
        Servo claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(pos);
    }

    /**
     * This is custom made to get the state of the paths to have movements without having to use the other codes.
     * It is a bit jank but it should work!
     * @author David Grieas - 14212 MetroBotics - former member of - 23403 C{}de C<>nduct<>rs
     */
    private String getState(@NonNull PathChain path) {
        for (int i = 0; i <= path.size(); i++) {
            if (path.getPath(i).isAtParametricStart()) {
                state = "Started" + i;
            }
            if (path.getPath(i).isAtParametricEnd()) {
                state = "Finished" + i;
            }
            if (i == path.size()) {
                state = "End";
            }
            assert state != null;
            if (state.equals("Started" + i)) {
                state = "Running" + i;
            }
        }
        return state;
    }
    private void pause(double ms) {
        int RealTime = (int) ms*1000;
        try {
            sleep(RealTime);
        } catch (InterruptedException e) {
            telemetry.addData("ERROR:", "COULDN'T SLEEP!! LINE 363");
            telemetry.update();
        }
    }
}
