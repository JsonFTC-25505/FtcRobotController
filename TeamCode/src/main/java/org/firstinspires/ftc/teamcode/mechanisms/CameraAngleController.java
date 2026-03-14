package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CameraAngleController {

    Servo servo;

    public static double servoDown = 0.48;
    public static double servoUp = 1;

    public void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "cameraController");

        //servo.setPosition(90);
    }

    public void lookDown(){
        servo.setPosition(servoDown);
    }

    public void lookUp(){
        servo.setPosition(servoUp);
    }

}
