// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// PID System For the JsonFTC (TalOS) team.
// Does PID For Values
//
// Methods:
// Name           | Arguments               | Action
// compute()      | double processVar       | PID output for a normal (non-angular) variable
// computeAngle() | double headingRad       | PID output for angles in radians (wraps error to [-pi, pi])
// setSetPoint()  | double setP             | Updates the setpoint
// reset()        | -                       | Resets integral/previous error and timer
//

package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.PIDConstants;

import java.lang.System;

public class PID {
    private double setPoint;
    private double previousError;
    private double integral;
    private long previousTimeMs;

    // Optional clamps (tune as you like)
    private double outMin = -1.0, outMax = 1.0;
    private double iMin = -0.5, iMax = 0.5;

    public PID(double setP) {
        setPoint = setP;
        reset();
    }

    public void setSetPoint(double setP) {
        this.setPoint = setP;
    }

    public void reset() {
        previousError = 0.0;
        integral = 0.0;
        previousTimeMs = System.currentTimeMillis();
    }

    /** For normal (non-angular) variables */
    public double compute(double processVar) {
        double dt = (System.currentTimeMillis() - previousTimeMs) / 1000.0; // seconds
        if (dt <= 1e-3) dt = 1e-3;

        double error = setPoint - processVar;

        double pOut = PIDConstants.Kp * error;

        integral = Range.clip(integral + error * dt, iMin, iMax);
        double iOut = PIDConstants.Ki * integral;

        double dOut = PIDConstants.Kd * (error - previousError) / dt;

        previousError = error;
        previousTimeMs = System.currentTimeMillis();

        return Range.clip(pOut + iOut + dOut, outMin, outMax);
    }

    /** For headings in radians (wrap error to [-pi, pi]) */
    public double computeAngle(double headingRad) {
        double dt = (System.currentTimeMillis() - previousTimeMs) / 1000.0;
        if (dt <= 1e-3) dt = 1e-3;

        double error = angleWrap(setPoint - headingRad);

        double pOut = PIDConstants.Kp * error;

        integral = Range.clip(integral + error * dt, iMin, iMax);
        double iOut = PIDConstants.Ki * integral;

        double dOut = PIDConstants.Kd * (error - previousError) / dt;

        previousError = error;
        previousTimeMs = System.currentTimeMillis();

        return Range.clip(pOut + iOut + dOut, outMin, outMax);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}

