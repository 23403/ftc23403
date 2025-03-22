package org.firstinspires.ftc.teamcode.variables.constants;

import xyz.nin1275.enums.Calibrates;
import xyz.nin1275.utils.Calibrate;

public class MConstants {
    public static boolean startUp = true;
    public static int eaStartPos1 = 0;
    public static int eaStartPos2 = 0;
    static {
        // Calibration Settings
        Calibrate.calibrates = Calibrates.ONLY_AUTO;
        // Color Sensor Calibration
        // Pick Up
        Calibrate.ColorRangeSensor.PickUp.Distance = 62.769500290720664;
        // Red Sample
        Calibrate.ColorRangeSensor.PickUp.RedSample.R = 41;
        Calibrate.ColorRangeSensor.PickUp.RedSample.G = 67;
        Calibrate.ColorRangeSensor.PickUp.RedSample.B = 65;
        Calibrate.ColorRangeSensor.PickUp.RedSample.A = 57;
        // Blue Sample
        Calibrate.ColorRangeSensor.PickUp.BlueSample.R = 32;
        Calibrate.ColorRangeSensor.PickUp.BlueSample.G = 65;
        Calibrate.ColorRangeSensor.PickUp.BlueSample.B = 74;
        Calibrate.ColorRangeSensor.PickUp.BlueSample.A = 57;
        // Yellow Sample
        Calibrate.ColorRangeSensor.PickUp.YellowSample.R = 48;
        Calibrate.ColorRangeSensor.PickUp.YellowSample.G = 85;
        Calibrate.ColorRangeSensor.PickUp.YellowSample.B = 66;
        Calibrate.ColorRangeSensor.PickUp.YellowSample.A = 66;
        // Grabbed
        Calibrate.ColorRangeSensor.Grabbed.Distance = 6.357584495013662;
        // Red Sample
        Calibrate.ColorRangeSensor.Grabbed.RedSample.R = 2427;
        Calibrate.ColorRangeSensor.Grabbed.RedSample.G = 1598;
        Calibrate.ColorRangeSensor.Grabbed.RedSample.B = 1171;
        Calibrate.ColorRangeSensor.Grabbed.RedSample.A = 1278;
        // Blue Sample
        Calibrate.ColorRangeSensor.Grabbed.BlueSample.R = 426;
        Calibrate.ColorRangeSensor.Grabbed.BlueSample.G = 928;
        Calibrate.ColorRangeSensor.Grabbed.BlueSample.B = 2483;
        Calibrate.ColorRangeSensor.Grabbed.BlueSample.A = 1278;
        // Yellow Sample
        Calibrate.ColorRangeSensor.Grabbed.YellowSample.R = 2111;
        Calibrate.ColorRangeSensor.Grabbed.YellowSample.G = 3203;
        Calibrate.ColorRangeSensor.Grabbed.YellowSample.B = 953;
        Calibrate.ColorRangeSensor.Grabbed.YellowSample.A = 2089;
    }
}
