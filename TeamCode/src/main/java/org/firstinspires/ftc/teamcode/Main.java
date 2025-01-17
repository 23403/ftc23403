package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Main", group="ftc23403")
public class Main extends LinearOpMode {

    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftFrontDrive;
    private DcMotor turnArm;
    private DcMotor extendArm;
    private Servo hangServo;
    private CRServo clawServo;
    private CRServo clawServo1;

    private int taLimitHigh;
    private int taLimitLow;
    private int taSP;
    private int taSPM;
    private int taSPL;
    private boolean sp;
    private double wheelSpeed;
    private double extendArmSpeed;
    private int taPL;
    private int taPLM;
    private int taPLL;
    private boolean pl;
    private boolean rtl;

    /**
     * This OpMode offers POV (point-of-view) style TeleOp control for a direct drive robot.
     *
     * In this POV mode, the left joystick (up and down) moves the robot forward and back, and the
     * right joystick (left and right) spins the robot left (counterclockwise) and right (clockwise).
     */

    private int teMax = -2300; //upp
    private int teMin = -50; //down

    @Override
    public void runOpMode() {
        Variables variables = new Variables();
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        turnArm = hardwareMap.get(DcMotor.class, "TurnArm");
        extendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
        hangServo = hardwareMap.get(Servo.class, "hook");
        clawServo = hardwareMap.get(CRServo.class, "claw");
        clawServo1 = hardwareMap.get(CRServo.class, "claw1");

        //Turn Arm Limits
        taLimitHigh = variables.taLimitHigh;
        taLimitLow = variables.taLimitLow;
        // starting POS
        taSP = variables.taSP;
        taSPM = variables.taSPM;
        taSPL = variables.taSPL;
        sp = variables.sp;
        wheelSpeed = variables.wheelSpeed;
        extendArmSpeed = variables.extendArmSpeed;
        // pickup low POS
        taPL = variables.taPL;
        taPLM = variables.taPLM;
        taPLL = variables.taPLL;
        pl = variables.pl;
        rtl = false;

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        // hang arm reset
        hangServo.setPosition(0);
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                int taPOS = turnArm.getCurrentPosition();
                int eaPOS = extendArm.getCurrentPosition();
                // Hang extend turn arm
                // int haPOS = hangExtendArm.getCurrentPosition();
                // hang arm servo
                double hsPOS = hangServo.getPosition();
                // turnArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // turnArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // left wheels
                leftBackDrive.setPower((gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                leftFrontDrive.setPower((-gamepad1.left_stick_x + (gamepad1.left_stick_y - gamepad1.right_stick_x)) * wheelSpeed);
                // right wheels
                rightFrontDrive.setPower((gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) * wheelSpeed);
                rightBackDrive.setPower((-gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) * wheelSpeed);
                // extend arm code/limits DAVID
        /*
        if (gamepad1.dpad_up) {
          extendArm.setPower(-extendArmSpeed);
        } else if (gamepad1.dpad_down) {
          extendArm.setPower(extendArmSpeed);
        } else if (!gamepad1.dpad_up || !gamepad1.dpad_down) {
          extendArm.setPower(0);
        }
        */
                // Slider motor control AYTEN
                int sliderPosition = extendArm.getCurrentPosition();
                if (gamepad1.dpad_up && sliderPosition > teMax) {
                    extendArm.setPower(-0.8);
                } else if (gamepad1.dpad_down && sliderPosition < teMin) {
                    extendArm.setPower(0.8);
                } else {
                    extendArm.setPower(0);
                }
                // claw servo
                if (gamepad1.left_bumper) {
                    clawServo.setPower(1);
                    clawServo1.setPower(-1);
                } else if (gamepad1.right_bumper){
                    clawServo.setPower(-1);
                    clawServo1.setPower(1);
                } else {
                    clawServo.setPower(0);
                    clawServo1.setPower(0);
                }
                // turn arm code/limits
                if (gamepad1.right_stick_y < 0) {
                    turnArm.setPower(gamepad1.right_stick_y);
                } else if (gamepad1.right_stick_y > 0) {
                    turnArm.setPower(gamepad1.right_stick_y);
                } else {
                    turnArm.setPower(0);
                }
        /*
        if (taPOS > taLimitHigh && taPOS < taLimitLow) {
          turnArm.setPower(gamepad1.right_stick_y);
        } else if(taPOS < taLimitLow) {
          if (gamepad1.right_stick_y > 0) {
            turnArm.setPower(gamepad1.right_stick_y);
          } else {
            turnArm.setPower(0);
          }
        } else if(taPOS > taLimitHigh) {
          if (gamepad1.right_stick_y < 0) {
            turnArm.setPower(gamepad1.right_stick_y);
          } else {
            turnArm.setPower(0);
          }
        } else {
          turnArm.setPower(0);
        }
        */
                // rt to pickup pos
                if(gamepad1.right_trigger > 0) {
                    rtl = false;
                }
                if (rtl) {
                    if (taPOS != taPL && taPOS != taPLM && taPOS != taPLL) {
                        if (taPOS < taPL) {
                            turnArm.setPower(0.7);
                        } else if (taPOS > taSP) {
                            turnArm.setPower(-0.7);
                        }
                    } else {
                        rtl = false;
                    }
                }
                // starting pos
                if (!sp) {
                    if (taPOS != taSP && taPOS != taSPM && taPOS != taSPL) {
                        if (taPOS < taSP) {
                            turnArm.setPower(0.7);
                        } else if (taPOS > taSP) {
                            turnArm.setPower(-0.7);
                        }
                    } else {
                        sp = true;
                    }
                }
                // hangArm
                if(gamepad1.y) {
                    hangServo.setPosition(0.4);
                } else if(gamepad1.a) {
                    hangServo.setPosition(0);
                }
                if (gamepad1.right_stick_y > 0) {
                    turnArm.setPower(gamepad1.right_stick_y);
                } else if (gamepad1.right_stick_y < 0) {
                    turnArm.setPower(gamepad1.right_stick_y);
                } else {
                    turnArm.setPower(0);
                }

        /* limits code
        if (taPOS > TALimitHigh && taPOS < TALimitLow) {
          turnArm.setPower(gamepad1.right_stick_y);
        } else if(taPOS < TALimitLow) {
          if (gamepad1.right_stick_y > 0) {
            turnArm.setPower(gamepad1.right_stick_y);
          } else {
            turnArm.setPower(0);
          }
        } else if(taPOS > TALimitHigh) {
          if (gamepad1.right_stick_y < 0) {
            turnArm.setPower(gamepad1.right_stick_y);
          } else {
            turnArm.setPower(0);
          }
        } else {
          turnArm.setPower(0);
        }
        */

                telemetry.addData("LeftBackDrive", leftBackDrive.getPower());
                telemetry.addData("LeftFrontDrive", leftFrontDrive.getPower());
                telemetry.addData("RightBackDrive", rightBackDrive.getPower());
                telemetry.addData("RightFrontDrive", rightFrontDrive.getPower());
                telemetry.addData("Turn Arm Position:", taPOS);
                telemetry.addData("Extend Arm Position:", eaPOS);
                telemetry.addData("Hang Servo", hsPOS);
                telemetry.update();
            }
        }
    }
}
