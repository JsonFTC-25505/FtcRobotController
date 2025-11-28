package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivers.MPU6050;

@TeleOp(name = "MPU6050 Test", group = "Test")
public class GyroTest extends LinearOpMode {

    private MPU6050 imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(MPU6050.class, "imu");

        telemetry.addLine("MPU6050 initialized, waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            MPU6050.Sample s = imu.readSample();

            telemetry.addData("Accel (g)", "x=%.2f y=%.2f z=%.2f",
                    s.axG, s.ayG, s.azG);
            telemetry.addData("Gyro (dps)", "x=%.1f y=%.1f z=%.1f",
                    s.gxDps, s.gyDps, s.gzDps);
            telemetry.addData("Temp (C)", "%.2f", s.tempC);
            telemetry.update();
        }
    }
}
