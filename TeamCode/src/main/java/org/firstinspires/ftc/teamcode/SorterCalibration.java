package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.CameraAngleController;

@Config
@Autonomous(name="SorterCal", group="Tests")
public class SorterCalibration extends OpMode {

    public static double degree = 0;
    Servo sorter;

    @Override
    public void init() {
        sorter = hardwareMap.get(Servo.class, "sorterServo");
    }

    @Override
    public void loop() {
        sorter.setPosition(degree);
    }
}
