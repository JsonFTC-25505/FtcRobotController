// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// Main TeleConroll System For the JsonFTC (TalOS) team.
// Currently: Recognises april tags and "lock" into them
// To-DO:
// - Make Search For AprilTag Mode (also needs to be implemented on teleOP so i will do it in its own class.)
// - Make the shorting Code based of april tags
//

package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModes.PIDController.PID;
import org.firstinspires.ftc.teamcode.drivers.MPU6050;

@TeleOp(name = "Main TeleOP")
public class MainTeleOP extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;

    private double speedScale = 0.5;

    private PID headingPID;
    private double headingRad = 0.0;
    private double targetHeadingRad = 0.0;

    private double gyroBiasDps = 0.0; // if you calibrate
    private boolean headingHoldActive = false;

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

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        throwing = false;
        intake = false;

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // Get our MPU device from the hardwareMap (name must match RC config)
        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize();

        timer.reset();
        calibrateGyroZ();
        waitForStart();
        headingTimer.reset();
        updateHeadingFromGyro();              // get a first sample
        targetHeadingRad = headingRad;
        headingPID = new PID(targetHeadingRad);
        headingPID.reset();

        while (opModeIsActive()) {
            updateHeadingFromGyro();
            gamepadInput();
            mecanumDrive();
            doTelemetry();
        }
    }

    private double deadband(double v) {
        return Math.abs(v) > 0.05 ? v : 0.0;
    }

    private void updateHeadingFromGyro() {
        MPU6050.Sample s = imu.readSample();

        double dt = headingTimer.seconds();   // seconds since last call
        headingTimer.reset();
        if (dt < 1e-4) dt = 1e-4;

        double gzDps = s.gzDps - gyroBiasDps;        // deg/s
        double gzRadPerSec = Math.toRadians(gzDps);  // rad/s

        headingRad = angleWrap(headingRad - gzRadPerSec * dt);
    }

    private void mecanumDrive() {
        double y = -deadband(gamepad1.left_stick_y);
        double x = deadband(gamepad1.left_stick_x) * 1.1;
        double rxManual = deadband(gamepad1.right_stick_x);

        boolean driverTurning = Math.abs(rxManual) > 0.05;
        boolean driverMoving  = (Math.abs(x) + Math.abs(y)) > 0.05;

        double rx; // final rotation command used by mecanum


        if (driverTurning) {
            rx = rxManual;

            // update target continuously while turning
            targetHeadingRad = headingRad;
            headingPID.setSetPoint(targetHeadingRad);

            headingHoldActive = false; // we are not holding
        }
        else if (driverMoving) {
            // entering heading-hold for the first time -> reset once
            if (!headingHoldActive) {
                targetHeadingRad = headingRad;
                headingPID.setSetPoint(targetHeadingRad);
                headingPID.reset();
                headingHoldActive = true;
            }

            double correction = headingPID.computeAngle(headingRad);

            double maxAutoTurn = 0.6;
            rx = Range.clip(correction, -maxAutoTurn, maxAutoTurn);
        }
        else {
            // stopped: just lock target to current, but don't spam reset every loop
            rx = 0.0;

            targetHeadingRad = headingRad;
            headingPID.setSetPoint(targetHeadingRad);

            headingHoldActive = false;
        }

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
            intakeMotor.setPower(1.0);
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
    private void calibrateGyroZ() {
        telemetry.addLine("Calibrating gyro Z... DO NOT MOVE ROBOT");
        telemetry.update();

        double sum = 0.0;
        int samples = 300;

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

    private void doTelemetry() {
        telemetry.addData("Speed Scale", "%.2f", speedScale);
        telemetry.addData("Throwing", throwing);
        telemetry.addData("Dispense Power", "%.2f", dispensePower);
        telemetry.addData("Intake", intake);;

        telemetry.update();
    }
}
