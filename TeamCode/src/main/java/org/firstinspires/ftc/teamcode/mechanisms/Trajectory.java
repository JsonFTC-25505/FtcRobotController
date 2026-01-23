// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// Trajectory Calculator for Objects
//
// Methods:
// Name         | Arguments | Action
// compute() | int x    | Calculates & returns the 2 possible angles for the dispenser based on distance (x)
//

package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.teamcode.utils.AnglePair;

public class Trajectory {
    // WARNING!!! THIS IS DUMMY NUMBER! WONT WORK WITH THIS. ACTUAL VELOCITY NEEDS TO BE CALCULATED)
    float v = 1f;
    float g = 9.81f; // Gravity Constant (not final cause one day gravity might change :thumbs_upz:)
    final float y = 1.16f; // Tower Height Constant

    public Trajectory(){

    }

    public AnglePair compute(int x){
        float angle1;
        float angle2;

        float A = (g * x * x) / (2f * v * v);
        float B = (float) Math.sqrt(x * x - 4f * A * (y + A));

        angle1 = (float) Math.atan((x + B) / (2f * A));
        angle2 = (float) Math.atan((x - B) / (2f * A));

        return new AnglePair(angle1, angle2);
    }
}