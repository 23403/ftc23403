package org.firstinspires.ftc.teamcode.variables;
public class Variables {
    // pub variables
    public int taLimitHigh;
    public int taLimitLow;
    public int teLimitHigh;
    public int teLimitLow;
    public int taSP;
    public int taSPM;
    public int taSPL;
    public boolean sp;
    public int taPL;
    public int taPLM;
    public int taPLL;
    public boolean pl;
    public double wheelSpeed;
    public double extendArmSpeed;
    public int EaMaxConstant;
    public boolean taLimits;
    // access limits across places
    public Variables() {
        // misc
        this.wheelSpeed = 1;
        this.extendArmSpeed = 0.7;
        // Turn Arm Limits
        this.taLimits = false;
        this.taLimitHigh = 10000;
        this.taLimitLow = -10000;
        // extend arm limits
        this.teLimitHigh = -2730;
        this.teLimitLow = 80;
        // starting POS
        this.taSP = 263;
        this.taSPM = taSP - 10;
        this.taSPL = taSP + 10;
        this.sp = true;
        // pickup POS
        this.taPL = 15;
        this.taPLM = taPL - 10;
        this.taPLL = taPL + 10;
    }
}

// -1858