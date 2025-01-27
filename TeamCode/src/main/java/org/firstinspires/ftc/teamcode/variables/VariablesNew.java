package org.firstinspires.ftc.teamcode.variables;
public class VariablesNew {
    // pub variables
    public double wheelSpeed;
    public double extendArmSpeed;
    public double turnArmSpeed;
    // turnArm
    public boolean taLimits;
    public boolean taCorrection;
    public int taLimitHigh;
    public int taLimitLow;
    // extendArm
    public boolean eaLimits;
    public boolean eaCorrection;
    public int eaLimitHigh;
    public int eaLimitLow;
    // starting position
    public boolean sp;
    public int taSP;
    public int eaSP;
    // preset locations
    public int specimenLoc;
    public int submersalLoc;

    // access limits across places
    public VariablesNew() {
        // misc
        this.wheelSpeed = 1;
        this.extendArmSpeed = 0.2;
        this.turnArmSpeed = 0.3;
        // turn arm
        this.taLimits = false;
        this.taCorrection = false;
        this.taLimitHigh = 10000;
        this.taLimitLow = -10000;
        // extend arm
        this.eaLimits = false;
        this.eaCorrection = false;
        this.eaLimitHigh = -10000;
        this.eaLimitLow = 10000;
        // starting POS
        this.sp = false;
        this.taSP = 263;
        this.eaSP = 120;
        // preset locations
        this.specimenLoc = 1000;
        this.submersalLoc = -1000;
    }
}

// -1858