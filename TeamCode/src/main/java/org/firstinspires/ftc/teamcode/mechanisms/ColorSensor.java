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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.RGBBundle;

public class ColorSensor {

    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorS");
        colorSensor.setGain(8);
    }

    public DetectedColor geDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normR, normG, normB;
        normR = colors.red / colors.alpha;
        normG = colors.green / colors.alpha;
        normB = colors.blue / colors.alpha;

//        telemetry.addData("red", normR);
//        telemetry.addData("green", normG);
//        telemetry.addData("blue", normB);

        // WARNING! TS NEEDS CALIBRATION FROM THE ACTUAL BALLS
        // WARNING NO MORE CALCULATED :3333 :> :>
        if (normR > 1.9 && normG < 2.7 && normB > 1.8){
            return DetectedColor.PURPLE;
        } else if (normR < 1.8 && normG > 1.9 && normB > 2.25) {
            return DetectedColor.GREEN;
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
