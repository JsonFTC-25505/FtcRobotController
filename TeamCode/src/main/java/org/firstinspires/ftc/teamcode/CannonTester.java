package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.CannonController;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name="Cannon Tester", group="Tests")
public class CannonTester extends OpMode {

    public static double velocity = 0;
    CannonController cannonController = new CannonController();
    @Override
    public void init() {
        velocity = 0;

        cannonController.init(hardwareMap);
    }

    @Override
    public void loop() {
        cannonController.setVelocity(velocity);
        cannonController.loop();
    }
}
