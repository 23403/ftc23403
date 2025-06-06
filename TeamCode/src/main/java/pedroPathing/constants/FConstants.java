package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL_IMU;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.xMovement = 76.0046440113;
        FollowerConstants.yMovement = 44.9396642203;

        FollowerConstants.forwardZeroPowerAcceleration = -28.947587181978584;
        FollowerConstants.lateralZeroPowerAcceleration = -67.6236691398;

        FollowerConstants.mass = 13.3;

        FollowerConstants.centripetalScaling = 0.001;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;

        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndTimeoutConstraint = 50;

        // pid for translational
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(
                0.025,
                0,
                0.002,
                0
        ); // BASIC TUNING
        FollowerConstants.translationalPIDFFeedForward = 0.02;
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(
                0.15,
                0,
                0.01,
                0
        ); // SECONDARY TUNING
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.0005;
        // pid for heading
        FollowerConstants.headingPIDFCoefficients.setCoefficients(
                0.7,
                0,
                0.05,
                0
        ); // BASIC TUNING
        FollowerConstants.headingPIDFFeedForward = 0.01;
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(
                0,
                0,
                0,
                0
        ); // SECONDARY TUNING
        FollowerConstants.secondaryHeadingPIDFFeedForward = 0.0005;
        // pid for drive
        FollowerConstants.drivePIDFCoefficients.setCoefficients(
                0.0145,
                0,
                0.0025,
                0.6,
                0
        ); // BASIC TUNING
        FollowerConstants.drivePIDFFeedForward = 0.01;
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(
                0,
                0,
                0,
                0.6,
                0
        ); // SECONDARY TUNING
        FollowerConstants.secondaryDrivePIDFFeedForward = 0.01;
    }
}
