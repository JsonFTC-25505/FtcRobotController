package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CanononController {

    DcMotor canon;

    public void init(HardwareMap hardwareMap){
        canon = hardwareMap.get(DcMotor.class, "canon");
        canon.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        canon.setDirection(DcMotor.Direction.FORWARD);
    }

    public void canonize(){
        canon.setPower(0.8);
    }

    public void unCanonize(){
        canon.setPower(0);
    }
}
