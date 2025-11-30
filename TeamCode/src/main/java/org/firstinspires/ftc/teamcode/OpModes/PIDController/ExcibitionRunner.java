package org.firstinspires.ftc.teamcode.OpModes.PIDController;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drivers.MPU6050;

@TeleOp(name = "Excibition- 101")
public class ExcibitionRunner extends LinearOpMode {

    // PID gains (reuse your PIDConstants)
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    private static final double ROTATE_DEADBAND = 0.05; // stick deadband
    private static final double MAX_PID_TURN = 0.4;     // clamp PID output to [-0.4, 0.4]

    Drivetrain drivetrain = new Drivetrain();

    ElapsedTime timer = new ElapsedTime();        // for PID dt
    ElapsedTime headingTimer = new ElapsedTime(); // for gyro integration dt
    private double lastError = 0;

    private MPU6050 imu;                 // our MPU6050
    private double heading = 0.0;        // current heading estimate (radians)
    private double gyroBiasDps = 0.0;    // gyro Z bias (deg/s)
    private double targetHeading = 0.0;  // heading we want to hold (radians)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrain.init(hardwareMap);

        // IMU init
        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize();

        // Calibrate gyro while robot is still
        calibrateGyroZ();

        // Reset timers and heading
        timer.reset();
        headingTimer.reset();
        heading = 0.0;         // define current orientation as 0
        targetHeading = 0.0;   // hold this at the start

        waitForStart();

        // Reset again at actual start, just to be safe
        timer.reset();
        headingTimer.reset();
        heading = 0.0;
        targetHeading = 0.0;

        while (opModeIsActive()) {

            // --- Update heading from gyro ---
            updateHeadingFromGyro();

            // --- Driver inputs ---
            double leftStickY = -gamepad1.left_stick_y;  // forward is negative on stick
            double leftStickX =  gamepad1.left_stick_x;  // strafe
            double rightStickX = gamepad1.right_stick_x; // driver rotation command

            boolean driverRotating = Math.abs(rightStickX) > ROTATE_DEADBAND;

            double rotationCommand;

            if (driverRotating) {
                // Let the driver rotate the robot normally
                rotationCommand = rightStickX;

                // While the driver is rotating, we continuously
                // update the target heading so that when they let go,
                // we hold the NEW heading.
                targetHeading = heading;

                // Optional: reset PID integral so we don't have windup
                integralSum = 0;
                lastError = 0;

            } else {
                // No driver rotation -> use PID to hold targetHeading
                rotationCommand = PIDControl(targetHeading, heading);
            }

            // --- Drive the robot with mecanum ---
            drivetrain.moveRobot(leftStickY, leftStickX, rotationCommand);

            // --- Telemetry ---
            telemetry.addData("Target Heading (rad)", targetHeading);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Current Heading (rad)", heading);
            telemetry.addData("Current Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Rotation Cmd", rotationCommand);
            telemetry.update();
        }
    }

    /**
     * PID between reference and current state (angles in radians).
     */
    public double PIDControl(double reference, double state) {
        double error = angleWrap(reference - state);

        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3; // safety against div by zero
        timer.reset();

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);

        // Clamp so it behaves like a "virtual stick"
        output = Range.clip(output, -MAX_PID_TURN, MAX_PID_TURN);

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
     */
    private void calibrateGyroZ() {
        telemetry.addLine("Calibrating gyro Z... DO NOT MOVE ROBOT");
        telemetry.update();

        double sum = 0;
        int samples = 200; // ~1s at 5ms per sample

        for (int i = 0; i < samples; i++) {
            MPU6050.Sample s = imu.readSample();
            sum += s.gzDps;
            sleep(5);
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
        if (dt <= 0) dt = 1e-3;
        headingTimer.reset();

        MPU6050.Sample s = imu.readSample();

        // Remove bias
        double gzDps = s.gzDps - gyroBiasDps;

        // If yaw direction feels inverted, uncomment:
        // gzDps = -gzDps;

        double gzRadPerSec = Math.toRadians(gzDps);
        heading += gzRadPerSec * dt;
        heading = angleWrap(heading);
    }
}
