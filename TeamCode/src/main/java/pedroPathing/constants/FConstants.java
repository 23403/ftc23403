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

        FollowerConstants.xMovement = 46.7468496303;
        FollowerConstants.yMovement = 22.9086578708;

        FollowerConstants.forwardZeroPowerAcceleration = -21.5054019464;
        FollowerConstants.lateralZeroPowerAcceleration = -45.4943839364;

        FollowerConstants.mass = 9.0718474;

        FollowerConstants.centripetalScaling = 0.001;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;

        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndTimeoutConstraint = 50;

        // pid for translational
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(
                0.085,
                0,
                0.000818,
                0
        ); // BASIC TUNING
        FollowerConstants.translationalPIDFFeedForward = 0.02;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(
                0,
                0,
                0,
                0
        ); // SECONDARY TUNING
        FollowerConstants.secondaryTranslationalPIDFFeedForward = 0.0005;
        // pid for heading
        FollowerConstants.headingPIDFCoefficients.setCoefficients(
                0.8,
                0,
                0.0455,
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
                0.02,
                0,
                0.0000054,
                0.6,
                0
        ); // BASIC TUNING
        FollowerConstants.drivePIDFFeedForward = 0.01;
        FollowerConstants.useSecondaryDrivePID = false;
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
