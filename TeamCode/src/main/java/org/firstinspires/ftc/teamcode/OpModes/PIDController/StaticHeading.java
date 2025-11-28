package org.firstinspires.ftc.teamcode.OpModes.PIDController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.MPU6050;

@TeleOp(name = "Static Heading (MPU6050)")
public class StaticHeading extends LinearOpMode {
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    Drivetrain drivetrain = new Drivetrain();

    ElapsedTime timer = new ElapsedTime();        // for PID dt
    ElapsedTime headingTimer = new ElapsedTime(); // for gyro integration dt
    private double lastError = 0;

    private MPU6050 imu;          // our MPU6050
    private double heading = 0.0; // current heading estimate (radians)
    private double gyroBiasDps = 0.0; // gyro Z bias (deg/s)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap);

        // Get our MPU device from the hardwareMap (name must match RC config)
        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize();  // calls doInitialize() inside your driver

        // --- Gyro calibration ---
        calibrateGyroZ();

        double refrenceAngle = Math.toRadians(90); // 90 degrees in radians

        timer.reset();
        headingTimer.reset();
        heading = 0.0; // define current orientation as 0 at start

        waitForStart();

        while (opModeIsActive()) {
            updateHeadingFromGyro(); // integrate gyro to update `heading`

            telemetry.addData("Target IMU Angle (rad)", refrenceAngle);
            telemetry.addData("Current IMU Angle (rad)", heading);
            telemetry.addData("Current IMU Angle (deg)", Math.toDegrees(heading));

            double power = PIDControl(refrenceAngle, heading);
            drivetrain.power(power);

            telemetry.update();
        }
    }

    /**
     * Simple PID using angleWrap for angular error.
     */
    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error (rad)", error);

        double dt = timer.seconds();
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    /**
     * Wrap angle to [-π, π]
     */
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    /**
     * Calibrate gyro Z bias while robot is still.
     * Call this BEFORE waitForStart().
     */
    private void calibrateGyroZ() {
        telemetry.addLine("Calibrating gyro Z... DO NOT MOVE ROBOT");
        telemetry.update();

        double sum = 0;
        int samples = 200; // ~1s at 5ms per sample

        for (int i = 0; i < samples; i++) {
            MPU6050.Sample s = imu.readSample();
            sum += s.gzDps;
            sleep(5); // LinearOpMode.sleep
        }

        gyroBiasDps = sum / samples;

        telemetry.addData("Gyro Z bias (deg/s)", gyroBiasDps);
        telemetry.addLine("Calibration complete");
        telemetry.update();
    }

    /**
     * Integrate gyro Z to update heading (radians).
     */
    private void updateHeadingFromGyro() {
        double dt = headingTimer.seconds();
        headingTimer.reset();

        MPU6050.Sample s = imu.readSample();

        // Remove bias
        double gzDps = s.gzDps - gyroBiasDps;

        // If turning direction is inverted, flip sign:
        // gzDps = -gzDps;

        double gzRadPerSec = Math.toRadians(gzDps);
        heading += gzRadPerSec * dt;
        heading = angleWrap(heading);
    }
}
