package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.00196568667;
        TwoWheelConstants.strafeTicksToInches = 0.0019744407;
        TwoWheelConstants.forwardY = 6.5;
        TwoWheelConstants.strafeX = 0;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "rightFront";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "leftRear";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




