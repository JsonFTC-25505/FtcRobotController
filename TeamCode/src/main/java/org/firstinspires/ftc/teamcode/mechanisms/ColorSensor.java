// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// ColorSensor Detection System For the JsonFTC (TalOS) team.
// Detects Specified colors and returns them
//
// Methods:
// Name              | Arguments            | Action
// geDetectedColor() | Telemetry telemetry  | Gets the current detected color (BALL GREEN, BALL PURPLE, UNKNOWN)
// getCurrentColor() | -                    | Returns the current rgb values seen by the sensor
//


package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.RGBBundle;

@Config
public class ColorSensor {

    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        G,
        P,
        B, // black
        UNKNOWN
    }

    public static double PR = 1.9;
    public static double PG = 2.7;
    public static double PB = 1.8;

    public static double GR = 1.8;
    public static double GG = 1.9;
    public static double GB = 2.25;

    public static double BR = 0.5;
    public static double BG = 0.5;
    public static double BB = 0.5;

    public void init(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorS");
        colorSensor.setGain(8);

    }

    public DetectedColor getDetectedColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normR, normG, normB;
        normR = colors.red / colors.alpha;
        normG = colors.green / colors.alpha;
        normB = colors.blue / colors.alpha;

//        telemetry.addData("red", normR);
//        telemetry.addData("green", normG);
//        telemetry.addData("blue", normB);

        // WARNING! TS NEEDS CALIBRATION FROM THE ACTUAL BALLS
        // WARNING NO MORE, CALCULATED :3333 :> :>
        if (normR > PR && normG < PG && normB > PB){
            return DetectedColor.P;
        } else if (normR < GR && normG > GG && normB > GB) {
            return DetectedColor.G;
        } else if (normR < BR && normG < BG && normB < BB){
            return DetectedColor.B;
        }

        return DetectedColor.UNKNOWN;
    }

    public RGBBundle getCurrentColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normR, normG, normB;
        normR = colors.red / colors.alpha;
        normG = colors.green / colors.alpha;
        normB = colors.blue / colors.alpha;

        return new RGBBundle(normR, normG, normB);
    }
}
