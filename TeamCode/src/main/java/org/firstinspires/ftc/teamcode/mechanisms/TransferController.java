package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TransferController {

    private Servo servo;

    private double currentDegrees = 0.9; // start in the bottom

    public void init(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "transferServo");
        servo.setPosition(currentDegrees);
    }

    public void loop(Telemetry telemetry){
        servo.setPosition(currentDegrees);

        telemetry.addData("Transfer Servo Pos", servo.getPosition());
    }
    //87 pano
    // 93 kato

    public void lookDown(){
        currentDegrees = 0.9;
    }

    public void lookUp(){

        currentDegrees = 0.65;
    }

}