package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.00294587293;
        ThreeWheelIMUConstants.strafeTicksToInches = 0.00294193648;
        ThreeWheelIMUConstants.turnTicksToInches = 0.002;
        ThreeWheelIMUConstants.leftY = 6.5;
        ThreeWheelIMUConstants.rightY = -6.5;
        ThreeWheelIMUConstants.strafeX = 0;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftRear";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightRear";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




