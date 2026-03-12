// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// Main Autonomous System For the JsonFTC (TalOS) team.
// Ball chase version:
// - SEARCH: rotate until a valid ball is found
// - APPROACH: center on the ball and drive in while intaking
// - SECURE: keep driving a little after likely capture
// - HOLD: stop and keep light intake power
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

import org.firstinspires.ftc.teamcode.drivers.MPU6050;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.PID;
import org.firstinspires.ftc.teamcode.mechanisms.WebcamProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@Config
@Autonomous(name = "Main OP")
public class MainOP extends OpMode {

    // ========= Motor Config =========
    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;
    private double speedScale = 0.7;

    // ========= Old PID / IMU Config (kept for later use if needed) =========
    private PID headingPID;
    private double headingRad = 0.0;
    private double targetHeadingRad = 0.0;
    private double gyroBiasDps = 0.0;
    private MPU6050 imu;

    ElapsedTime headingTimer = new ElapsedTime();

    WebcamProcessor webcamProcessor = new WebcamProcessor();

    // ========= Ball Chase Config =========
    public static double BALL_CENTER_X = 320.0;      // 640x480 image center
    public static double BALL_X_DEADBAND = 14.0;     // px
    public static double BALL_TURN_GAIN = 0.0035;    // turn per pixel
    public static double MAX_BALL_TURN = 0.40;

    public static double SEARCH_TURN_POWER = 0.4;
    public static double APPROACH_POWER = 1.0;
    public static double ALIGN_ONLY_ERROR = 35.0;    // rotate first if too far off center
    public static double INTAKE_POWER = -1.0;

    public static double LOST_CLOSE_RADIUS = 1000.0;   // if blob vanishes after this radius, assume captured
    public static double SECURE_DRIVE_POWER = 0.12;
    public static double SECURE_TIME = 0.35;         // seconds

    private enum AutoState {
        SEARCH,
        APPROACH,
        SECURE,
        HOLD
    }

    private AutoState autoState = AutoState.SEARCH;
    private ElapsedTime stateTimer = new ElapsedTime();
    private double lastSeenTurnSign = 1.0;
    private double lastSeenRadius = 0.0;

    ColorSensor colorSensor = new ColorSensor();

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

        double dt = headingTimer.seconds();
        headingTimer.reset();
        if (dt < 1e-4) dt = 1e-4;

        double gzDps = s.gzDps - gyroBiasDps;
        double gzRadPerSec = Math.toRadians(gzDps);

        headingRad = angleWrap(headingRad - gzRadPerSec * dt);
    }

    private void initializeTelemetry() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
    }

    @Override
    public void init() {
        // ========= Motor Setup =========
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

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeTelemetry();

        // ========= ColorSensor =======

        colorSensor.init(hardwareMap);

        // ========= IMU Setup (optional for later use) =========
        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize();

        calibrateGyroZ();
        headingTimer.reset();
        updateHeadingFromGyro();
        targetHeadingRad = headingRad;
        headingPID = new PID(targetHeadingRad);
        headingPID.reset();

        // ========= State Init =========
        autoState = AutoState.SEARCH;
        stateTimer.reset();
        lastSeenTurnSign = 1.0;
        lastSeenRadius = 0.0;

        webcamProcessor.init(hardwareMap, telemetry);
    }

    private void setDrive(double drive, double turn) {
        double left = Range.clip(drive + turn, -1.0, 1.0) * speedScale;
        double right = Range.clip(drive - turn, -1.0, 1.0) * speedScale;

        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    private void setIntake(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    @Override
    public void loop() {
        webcamProcessor.update();

        ColorBlobLocatorProcessor.Blob ball = webcamProcessor.getBestBlob();
        webcamProcessor.blobTelemetry(ball);

        double drive = 0.0;
        double turn = 0.0;

        if (ball != null) {
            double errorPxNow = ball.getCircle().getX() - BALL_CENTER_X;
            lastSeenRadius = ball.getCircle().getRadius();

            if (Math.abs(errorPxNow) > 1e-3) {
                lastSeenTurnSign = Math.signum(errorPxNow);
            }
        }

        switch (autoState) {
            case SEARCH: {
                setIntake(0.0);

                if (ball != null) {
                    autoState = AutoState.APPROACH;
                    stateTimer.reset();
                    drive = 0.0;
                    turn = 0.0;
                } else {
                    turn = SEARCH_TURN_POWER * lastSeenTurnSign;
                }
                break;
            }

            case APPROACH: {
                setIntake(INTAKE_POWER);

                if (ball == null) {
                    if (colorSensor.getDetectedColor() != ColorSensor.DetectedColor.B && colorSensor.getDetectedColor() != ColorSensor.DetectedColor.UNKNOWN) {
                        autoState = AutoState.SECURE;
                        stateTimer.reset();
                    } else {
                        autoState = AutoState.SEARCH;
                    }
                    break;
                }

                double errorPx = ball.getCircle().getX() - BALL_CENTER_X;

                if (Math.abs(errorPx) < BALL_X_DEADBAND) {
                    errorPx = 0.0;
                }

                // Negated because your robot was turning the wrong way before
                turn = Range.clip(-errorPx * BALL_TURN_GAIN, -MAX_BALL_TURN, MAX_BALL_TURN);

                // Rotate first, only drive when centered enough
                if (Math.abs(errorPx) > ALIGN_ONLY_ERROR) {
                    drive = 0.0;
                } else {
                    drive = APPROACH_POWER;
                }
                break;
            }

            case SECURE: {
                setIntake(INTAKE_POWER);
                drive = SECURE_DRIVE_POWER;
                turn = 0.0;

                if (stateTimer.seconds() >= SECURE_TIME) {
                    autoState = AutoState.HOLD;
                }
                break;
            }

            case HOLD: {
                drive = 0.0;
                turn = 0.0;
                setIntake(-0.25); // light holding power
                break;
            }
        }

        setDrive(drive, turn);

        telemetry.addData("state", autoState);
        telemetry.addData("drive", drive);
        telemetry.addData("turn", turn);
        telemetry.addData("lastSeenRadius", lastSeenRadius);

        if (ball != null) {
            telemetry.addData("ballX", ball.getCircle().getX());
            telemetry.addData("ballY", ball.getCircle().getY());
            telemetry.addData("ballRadius", ball.getCircle().getRadius());
            telemetry.addData("ballErrorPx", ball.getCircle().getX() - BALL_CENTER_X);
            telemetry.addData("ballCircularity", ball.getCircularity());
            telemetry.addData("ballDensity", ball.getDensity());
        } else {
            telemetry.addLine("No valid ball detected");
        }

        telemetry.update();
    }
}