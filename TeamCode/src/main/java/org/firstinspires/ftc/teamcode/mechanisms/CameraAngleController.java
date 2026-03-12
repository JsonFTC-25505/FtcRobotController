package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class CameraAngleController {

    Servo servo;

    public void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "cameraController");

        servo.setPosition(90);
    }

    public void lookDown(){
        servo.setPosition(-90);
    }

    public void lookUp(){
        servo.setPosition(180);
    }

}
