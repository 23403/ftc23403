package org.firstinspires.ftc.teamcode.variables.old;
public class VariablesNewOld {
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
    public int feildLoc;
    public int basketLoc;

    // access limits across places
    public VariablesNewOld() {
        // misc
        this.wheelSpeed = 1;
        this.extendArmSpeed = 1;
        this.turnArmSpeed = 0.5;
        // turn arm
        this.taLimits = true;
        this.taCorrection = true;
        this.taLimitHigh = 919;
        this.taLimitLow = -253;
        // extend arm
        this.eaLimits = true;
        this.eaCorrection = true;
        this.eaLimitHigh = 2272;
        this.eaLimitLow = 0;
        // starting POS
        this.sp = false;
        this.taSP = 263;
        this.eaSP = 120;
        // preset locations
        this.feildLoc = -260;
        this.basketLoc = 717;
        this.specimenLoc = 420;
        this.submersalLoc = 200;
    }
}