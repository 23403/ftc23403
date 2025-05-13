package org.firstinspires.ftc.teamcode.variables.presets;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teleOp.MainV6;
import org.firstinspires.ftc.teamcode.utils.CustomPresets;

@Config("MainV6 Presets")
public class MainV6Presets {
    public static CustomPresets humanPlayer = new CustomPresets(
            MainV6.eaLimitLow,
            -1.0,
            -1.0,
            0.0,
            -1.0,
            0.42,
            0.96,
            -1.0);
    public static CustomPresets highBasket = new CustomPresets(
            33.6,
            -1.0,
            0.0,
            1.0,
            -1.0,
            0.6,
            0.8,
            -1.0);
    public static CustomPresets lowBasket = new CustomPresets(
            16.5,
            -1.0,
            0.0,
            1.0,
            -1.0,
            0.6,
            0.8,
            -1.0);
    public static CustomPresets transition = new CustomPresets(
            MainV6.eaLimitLow,
            1.0,
            1.0,
            0.0,
            1.0,
            0.5,
            0.18,
            0.52);
    public static CustomPresets preSpecimen = new CustomPresets(
            10,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            0.6,
            0.23,
            -1.0);
    public static CustomPresets scoreSpecimen = new CustomPresets(
            19.5,
            -1.0,
            -1.0,
            1.0,
            -1.0,
            0.6,
            0.23,
            -1.0);
    public static CustomPresets preHang = new CustomPresets(
            33.6,
            1.0,
            1.0,
            1.0,
            1.0,
            0.5,
            0.18,
            0.52);
    public static CustomPresets hang = new CustomPresets(
            MainV6.eaLimitLow,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0);
}