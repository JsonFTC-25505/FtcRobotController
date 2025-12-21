package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivers.MPU6050;

@TeleOp(name = "Main TeleOP (fixed mecanum - from AD-Studio)")
public class MainTeleOP extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;

    // Overall drive scale (left bumper = slow, right bumper = fast)
    private double speedScale = 0.5;

    private boolean throwing;
    private boolean intake;
    private double dispensePower;

    ElapsedTime timer = new ElapsedTime();        // for PID dt
    ElapsedTime headingTimer = new ElapsedTime(); // for gyro integration dt

    private double lastError = 0;
    private MPU6050 imu; // our MPU6050

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Typical FTC setup: reverse the left side so +power drives all wheels forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        throwing = false;
        intake = false;

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // Get our MPU device from the hardwareMap (name must match RC config)
        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize(); // calls doInitialize() inside your driver

        // --- Gyro calibration ---
//        calibrateGyroZ();

        timer.reset();
        headingTimer.reset();

        waitForStart();

        while (opModeIsActive()) {
            //updateHeadingFromGyro();

            gamepadInput();
            mecanumDrive();
            doTelemetry();

        }
    }

    private double deadband(double v) {
        return Math.abs(v) > 0.05 ? v : 0.0;
    }

    private void mecanumDrive() {
        double y = -deadband(gamepad1.left_stick_y);           // stick up = forward
        double x = deadband(gamepad1.left_stick_x) * 1.1;      // inverted strafe input
        double rx = deadband(gamepad1.right_stick_x);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double lf = (y + x + rx) / denominator;
        double lb = (y - x + rx) / denominator;
        double rf = (y - x - rx) / denominator;
        double rb = (y + x - rx) / denominator;

        frontLeft.setPower(lf * speedScale);
        backLeft.setPower(lb * speedScale);
        frontRight.setPower(rf * speedScale);
        backRight.setPower(rb * speedScale);
    }

    private void gamepadInput() {
        // Speed modes
        if (gamepad1.left_bumper) {
            speedScale = 0.4; // precision
        } else if (gamepad1.right_bumper) {
            speedScale = 1.0; // full speed
        } else {
            speedScale = 0.8; // default
        }

        // Example mechanism control (range -1..1). Replace with your motor/CRServo as needed.
        if (gamepad1.right_trigger > 0.1) {
            throwing = true;
            dispensePower = gamepad1.right_trigger; // 0..1
        } else {
            throwing = false;
            dispensePower = 0.0;
        }

        if (gamepad1.left_trigger > 0.1) {
            intake = true;
            intakeMotor.setPower(5);
        } else {
            intake = false;
            intakeMotor.setPower(0);
        }
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
//    private void calibrateGyroZ() {
//        telemetry.addLine("Calibrating gyro Z... DO NOT MOVE ROBOT");
//        telemetry.update();
//
//        double sum = 0;
//        int samples = 200; // ~1s at 5ms per sample
//
//        for (int i = 0; i < samples; i++) {
//            MPU6050.Sample s = imu.readSample();
//            sum += s.gzDps;
//            sleep(5); // LinearOpMode.sleep
//        }
//
//        gyroBiasDps = sum / samples;
//
//        telemetry.addData("Gyro Z bias (deg/s)", gyroBiasDps);
//        telemetry.addLine("Calibration complete");
//        telemetry.update();
//    }

    private void doTelemetry() {
        telemetry.addData("Speed Scale", "%.2f", speedScale);
        telemetry.addData("Throwing", throwing);
        telemetry.addData("Dispense Power", "%.2f", dispensePower);
        telemetry.addData("Intake", intake);;

        telemetry.update();
    }
}
