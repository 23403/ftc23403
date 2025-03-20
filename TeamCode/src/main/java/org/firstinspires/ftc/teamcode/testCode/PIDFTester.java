package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDFTester {
    private DcMotor motor;
    private boolean initialized = false;

    public final PIDFController pidf;
    private int target;

    public PIDFTester(DcMotor motor, int target, PIDFController.PIDCoefficients coefficients) {
        this.motor = motor;
        this.pidf = new PIDFController(coefficients, (position, velocity) -> 0);
        this.target = target;
    }

    // Getter for target
    public int getTarget() {
        return target;
    }

    // Setter for target with custom behavior:
    // It updates the PIDFController's targetPosition as well as the local target.
    public void setTarget(int value) {
        pidf.targetPosition = value;
        this.target = value;
    }

    public boolean run(TelemetryPacket p) {
        if (!initialized) {
            pidf.targetPosition = target;
            initialized = true;
        }

        int position = motor.getCurrentPosition();
        double power = pidf.update(position);

        p.put("Motor Info", "Target: " + target + "; Error " + (target - position) + "; Power: " + power);

        motor.setPower(power);

        return (position >= target - 50 && position <= target + 50);
    }

}
