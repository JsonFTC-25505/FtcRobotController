package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Pot Test", group = "Sensor")
public class PotTest extends LinearOpMode {

    private AnalogInput pot;

    @Override
    public void runOpMode() {
        // Map the analog input named "pot" in the Robot Config
        pot = hardwareMap.get(AnalogInput.class, "pot");

        telemetry.addLine("Pot Tester ready. Expect ~0–3.3V on AIN0/2.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double v = pot.getVoltage();     // 0..~3.3 V on REV hubs
            double norm = Range.clip(v / 3.3, 0.0, 1.0); // normalize to 0..1
            double angleDeg = norm * 270.0;  // for a 270° linear pot

            telemetry.addData("Voltage (V)", "%.3f", v);
            telemetry.addData("Normalized",  "%.3f", norm);
            telemetry.addData("Angle (deg)", "%.1f", angleDeg);
            telemetry.update();

            sleep(20); // ~50 Hz update
        }
    }
}
