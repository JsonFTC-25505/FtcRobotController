package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor;

@Config
@Autonomous(name = "Balls")
public class BallCalculator extends OpMode {

    ColorSensor colorSensor =  new ColorSensor();

    @Override
    public void init() {
        colorSensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        RGBBundle colors = colorSensor.getCurrentColor();

        telemetry.addLine(String.valueOf(colors.r));
        telemetry.addLine(String.valueOf(colors.g));
        telemetry.addLine(String.valueOf(colors.b));

        ColorSensor.DetectedColor currentBall = colorSensor.geDetectedColor(telemetry);

        telemetry.addLine(String.valueOf(currentBall));
        telemetry.update();



    }
}
