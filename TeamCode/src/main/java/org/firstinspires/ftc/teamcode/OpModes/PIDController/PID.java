package org.firstinspires.ftc.teamcode.OpModes.PIDController;
import org.firstinspires.ftc.teamcode.OpModes.PIDController.PIDConstants;

import java.security.Timestamp;
import java.lang.System;
import java.util.Date;

public class PID {
    private double setPoint;
    private double previousError;
    private double integral;
    private double error;
    private long previousTime;

    public PID(double setP){
        setPoint = setP;
        previousError = 0;
        integral = 0;
        previousTime = System.currentTimeMillis();
    }

    public double compute(double processVar){

        long dt = System.currentTimeMillis() - previousTime;
        error = setPoint - processVar;

        double POut = PIDConstants.Kp * error;

        integral += error * dt;

        double IOut = PIDConstants.Ki * integral;

        double DOut = PIDConstants.Kd * (error - previousError) / dt;

        previousError = error;
        previousTime = System.currentTimeMillis();

        return POut + IOut + DOut;

    }
}
