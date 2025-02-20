package metroLib.motors;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class motorUtils {
    public static void setBrakeMode(List<DcMotor> motors,boolean enabled) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(enabled? DcMotor.ZeroPowerBehavior.BRAKE: DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public static void resetEncoders(List<DcMotor> motors) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public static void calibrateMotors(List<DcMotor> motors, List<Integer> positions, Telemetry telemetry) {
        int i = 0;
        for (DcMotor motor : motors) {
            i++;
            resetEncoders(List.of(motor));
            motor.setTargetPosition(-positions.get(i));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Thread loopThread = new Thread(() -> {
                while (!motor.isBusy()) {
                    resetEncoders(List.of(motor));
                    telemetry.addData("RESETTING", "DONE!");
                    telemetry.update();
                }
            });
            loopThread.start();
            telemetry.addData("RESETTING", "POSITIONS");
            telemetry.update();
        }
    }
}
