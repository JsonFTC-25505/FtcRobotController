package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class CannonController {

    public DcMotorEx canon;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    public void init(HardwareMap hardwareMap){
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F );

        canon = hardwareMap.get(DcMotorEx.class, "canon");
        canon.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        canon.setDirection(DcMotorEx.Direction.FORWARD);
        canon.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void loop(){
        updatePIDFCoEf();
        canon.setVelocity(curTargetVelocity);

        double curVelocity = canon.getVelocity();
        double error = curTargetVelocity - curVelocity;
    }

    public void switchSpeed() {
        if (curTargetVelocity == highVelocity)
            curTargetVelocity = lowVelocity;
        else
            curTargetVelocity = highVelocity;
    }

    public void upStepIndex(){
        stepIndex = (stepIndex + 1) % stepSizes.length;
    }

    public void incrementF(){
        F += stepSizes[stepIndex];
    }

    public void decrementF(){
        F -= stepSizes[stepIndex];
    }

    public void incrementP(){
        P += stepSizes[stepIndex];
    }

    public void decrementP(){
        P -= stepSizes[stepIndex];
    }

    public void updatePIDFCoEf(){
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        canon.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public void canonize(){
        canon.setPower(0.8);
    }

    public void unCanonize(){
        canon.setPower(0);
    }
}