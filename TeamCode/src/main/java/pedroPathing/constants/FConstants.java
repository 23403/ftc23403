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

        FollowerConstants.mass = 9.0718474;

        FollowerConstants.xMovement = 46.7468496303;
        FollowerConstants.yMovement = 22.9086578708;

        FollowerConstants.forwardZeroPowerAcceleration = -21.5054019464;
        FollowerConstants.lateralZeroPowerAcceleration = -45.4943839364;

        /**
         ** STEPS FOR BASIC PID TUNING **
         *** HAVE SECONDARY TUNING OFF WHEN DOING THIS STEP ***
         * P - For power which will overshoot. You want to have very little overshoot but still have power
         * I - For overshooting. Only use this once in a lifetime if you can't get D working properly for some reason.
         * D - For de-overshooting. When you increase this you remove the overshoot but too much D value will undershoot which is bad
         ** STEPS FOR SECONDARY/ADVANCED PID TUNING **
         *** USE THE GRAPH WHEN TUNING! ***
         * P - For power which will overshoot. You want to get everything as close to the 0 value as possible on the graph
         * I - For overshooting. Only use this once in a lifetime if you can't get D working properly for some reason.
         * D - For de-overshooting. You want to get everything as close to the 0 value as possible on the graph
        **/

        // pid for translational
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.085,0,0.000818,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0,0,0,0); // SECONDARY TUNING
        // pid for heading
        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.8,0,0.0455,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0,0,0,0); // SECONDARY TUNING
        // pid for drive
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0000027,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0,0,0,0.6,0); // SECONDARY TUNING

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.001;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
