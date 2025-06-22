package org.firstinspires.ftc.teamcode.variables.presets;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teleOp.MainV6;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;

@Config("MainV6 Presets")
public class MainV6Presets {
    public static CustomPresets humanPlayer = new CustomPresets(
            MainV6.eaLimitLow,
            1.0,
            -1.0,
            0.0,
            0.0,
            0.6,
            0.1,
            0.0,
            0.0);
    public static CustomPresets highBasket = new CustomPresets(
            23,
            -1.0,
            0.0,
            1.0,
            -1.0,
            0.9,
            0.55,
            1.0,
            -1.0);
    public static CustomPresets lowBasket = new CustomPresets(
            7,
            -1.0,
            0.0,
            1.0,
            -1.0,
            0.9,
            0.55,
            1.0,
            -1.0);
    public static CustomPresets transition = new CustomPresets(
            MainV6.eaLimitLow,
            1.0,
            1.0,
            0.0,
            0.0,
            0.5,
            0.1,
            0.0,
            0.0);
    public static CustomPresets reTurn = new CustomPresets(
            MainV6.eaLimitLow,
            1.0,
            1.0,
            0.0,
            1.0,
            0.5,
            0.18,
            0.0,
            0.0);
    public static CustomPresets preSpecimen = new CustomPresets(
            MainV6.eaLimitLow,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            0.6,
            0.55,
            1.0,
            -1.0);
    public static CustomPresets scoreSpecimen = new CustomPresets(
            -1,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            0.75,
            -1.0,
            -1.0);
    public static CustomPresets slidesOut = new CustomPresets(
            -1,
            0.0,
            0.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            0.0,
            -1.0);
    public static CustomPresets subThrow = new CustomPresets(
            -1,
            0.0,
            0.0,
            -1.0,
            0.5,
            -1.0,
            -1.0,
            0.0,
            -1.0);
}