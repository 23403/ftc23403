package org.firstinspires.ftc.teamcode.variables.old;

import com.acmerobotics.dashboard.config.Config;

@Config("MainV2")
public class ConfigVariables {
    public static double wristCpos = 1;
    // 0.5 low pos
    // 1 high pos
    public static double clawCpos = 0.47;
    // claw max pos 0.3 to 0.54
    // 0.48 is the close/tight pos
    // 0.54 is close
    // 0.3 is open pos
    // 0.47 is perfect pos
    public static int taCpos = 180;
    public static int eaCpos1 = 0;
    public static int eaCpos2 = 0;
    // misc
    public static double turnArmSpeed = 0.5;
    public static double extendArmSpeed = 1;
    public static double wheelSpeed = 1;
    // turn arm
    public static boolean taLimits = true;
    public static boolean taCorrection = true;
    public static int taLimitHigh = 919;
    public static int taLimitLow = -253;
    // extend arm
    public static boolean eaLimits = true;
    public static boolean eaCorrection = true;
    public static int eaLimitHigh1 = 2272;
    public static int eaLimitHigh2 = 2272;
    public static int eaLimitLow1 = 0;
    public static int eaLimitLow2 = 0;
    // starting POS
    public static int taSP = 180;
    // preset locations
    public static int taSpecimenLoc = 702;
    public static int eaSpecimenLoc1 = 451;
    public static int eaSpecimenLoc2 = 447;
    public static int submersalLoc = 200;
    public static int feildLoc = -260;
    public static int basketLoc = 717;
}
