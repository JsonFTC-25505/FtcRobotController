// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// Main Autonomous System For the JsonFTC (TalOS) team.
// Currently: Recognises april tags and "lock" into them
// To-DO:
// - Make Search For AprilTag Mode (also needs to be implemented on teleOP so i will do it in its own class.)
// - Make the moving part of the roboto so it can complete mission.
//

package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.PID;
import org.firstinspires.ftc.teamcode.drivers.MPU6050;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "Main OP")
public class MainOP extends OpMode {

    // ========= MotoConfig =========
    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;
    private double speedScale = 0.7;

    // ========= PID Config =========
    private PID headingPID;
    private double headingRad = 0.0;
    private double targetHeadingRad = 0.0;
    private double gyroBiasDps = 0.0;
    private double lastError = 0;
    boolean tracking = false;
    public static double BEARING_DEADBAND = 0.03; // rad ~ 1.7deg
    public static double maxAutoTurn = 0.6;

    private MPU6050 imu;

    ElapsedTime timer = new ElapsedTime();        // for PID dt
    ElapsedTime headingTimer = new ElapsedTime(); // for gyro integration dt

    // ========= AprilTag Config =========
    public static int tagId = 24;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // ========= ColorSensor Config =========

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }


    private void calibrateGyroZ() {
        telemetry.addLine("Calibrating gyro Z... DO NOT MOVE ROBOT");
        telemetry.update();

        double sum = 0.0;
        int samples = 300;

        for (int i = 0; i < samples; i++) {
            MPU6050.Sample s = imu.readSample();
            sum += s.gzDps;
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        gyroBiasDps = sum / samples;

        telemetry.addData("Gyro Z bias (deg/s)", gyroBiasDps);
        telemetry.addLine("Calibration complete :)");
        telemetry.update();
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

    private void initializeTelemetry() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
    }

    @Override
    public void init()
    {
        // ========= Motor Setup =========

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        initializeTelemetry();

        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize();

        timer.reset();
        calibrateGyroZ();
        headingTimer.reset();
        updateHeadingFromGyro();              // get a first sample
        targetHeadingRad = headingRad;
        headingPID = new PID(targetHeadingRad);
        headingPID.reset();


        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection det = aprilTagWebcam.getTagById(tagId);
        aprilTagWebcam.aprilTagTelemetry(det);

        updateHeadingFromGyro();

        double rx = 0.0;

        if (det != null) {
            double bearing = det.ftcPose.bearing; // radians

            // deadband to avoid jitter when almost centered
            if (Math.abs(bearing) < BEARING_DEADBAND) bearing = 0.0;

            // if this turns the wrong way, flip the sign (+bearing instead of -bearing)
            targetHeadingRad = angleWrap(headingRad - bearing);

            headingPID.setSetPoint(targetHeadingRad);

            // reset only ONCE when we first acquire the tag (or after being lost)
            if (!tracking) {
                headingPID.reset();
                tracking = true;
            }

            double correction = headingPID.computeAngle(headingRad);
            rx = Range.clip(correction, -maxAutoTurn, maxAutoTurn);

        } else {
            tracking = false;
            rx = 0.0; // or a small constant to SEARCH, like 0.15
        }

        frontLeft.setPower( rx * speedScale);
        backLeft.setPower(  rx * speedScale);
        frontRight.setPower(-rx * speedScale);
        backRight.setPower( -rx * speedScale);

        telemetry.addData("headingRad", headingRad);
        telemetry.addData("targetHeadingRad", targetHeadingRad);
        if (det != null) telemetry.addData("bearing", det.ftcPose.bearing);
        telemetry.addData("rx", rx);
    }
}