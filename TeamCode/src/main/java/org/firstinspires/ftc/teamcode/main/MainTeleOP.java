// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// Main TeleControll System For the JsonFTC (TalOS) team.
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

import org.firstinspires.ftc.teamcode.mechanisms.CannonController;
import org.firstinspires.ftc.teamcode.mechanisms.HoodControl;
import org.firstinspires.ftc.teamcode.mechanisms.PID;
import org.firstinspires.ftc.teamcode.drivers.MPU6050;
import org.firstinspires.ftc.teamcode.mechanisms.Sorter;
import org.firstinspires.ftc.teamcode.mechanisms.TransferController;
import org.firstinspires.ftc.teamcode.mechanisms.WebcamProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Objects;

@TeleOp(name = "Main TeleOP")
public class MainTeleOP extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeMotor;

    private final double speedScale = 0.8;

    private PID headingPID;
    private double headingRad = 0.0;
    private double targetHeadingRad = 0.0;

    private double gyroBiasDps = 0.0;
    private boolean headingHoldActive = false;

    private boolean throwing;
    private boolean intake;
    private double dispensePower;

    public enum Balls {
        PPG,
        PGP,
        GPP
    }

    public String teamColor = "Blue";
    public double towerDistance;

    // Just here to remember
    public double maxShotingRange = 228;

    public boolean lowMode = false;

    Balls ballCombination;

    WebcamProcessor webcamProcessor = new WebcamProcessor();

    HoodControl hoodControl = new HoodControl();

    CannonController cannonController = new CannonController();

    ElapsedTime timer = new ElapsedTime();        // for PID dt
    ElapsedTime headingTimer = new ElapsedTime(); // for gyro integration dt

    Sorter sorter = new Sorter();

    TransferController transferController = new TransferController();

    private MPU6050 imu; // our MPU6050

    private boolean lastShootButton = false;
    private boolean lastSeekBlackNowButton = false;
    private boolean lastSeekNextBlackButton = false;

    private boolean lastLowModeButton = false;
    private boolean lastTeamColorButton = false;

    private boolean intakeReverse = false;

    private void initializeTelemetry() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
    }

    private void initializeMotors(){
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
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        throwing = false;
        intake = false;
    }

    private void initializeSensors(){
        // Get our MPU device from the hardwareMap (name must match RC config)
        imu = hardwareMap.get(MPU6050.class, "imu");
        imu.initialize();

        timer.reset();
        calibrateGyroZ();
        webcamProcessor.init(hardwareMap, telemetry);
    }

    private void initializeControllers(){
        hoodControl.init(hardwareMap, telemetry);
        cannonController.init(hardwareMap);
        sorter.init(hardwareMap);
        transferController.init(hardwareMap);
    }

    private void setupPID(){
        headingTimer.reset();
        updateHeadingFromGyro();
        targetHeadingRad = headingRad;
        headingPID = new PID(targetHeadingRad);
        headingPID.reset();
    }

    @Override
    public void runOpMode() {
        // Initializers
        initializeTelemetry();
        initializeMotors();
        initializeSensors();
        initializeControllers();

        waitForStart();

        setupPID();

        while (opModeIsActive()) {
            updateHeadingFromGyro();
            gamepadInput();
            mecanumDrive();
            cannonController.loop(lowMode);
            sorter.loop(telemetry, gamepad1);
            transferController.loop(telemetry);
            webcamProcessor.update();
//            if (lowMode && !intake){
//                intakeMotor.setPower(0.4);
//            }
            if (ballCombination == null) {
                ballCombination = getBallCombination();
            }
            if (getTowerDistance() != null)
                towerDistance = getTowerDistance().ftcPose.range;
            doTelemetry();
        }
    }

    private Balls getBallCombination(){
        AprilTagDetection tag = webcamProcessor.getTagByPrefix("Obelisk_");
        if (tag == null) { return null; }
        String comp = tag.metadata.name.split("_")[1];
        return Balls.valueOf(comp);
    }

    private AprilTagDetection getTowerDistance(){
        AprilTagDetection tag = webcamProcessor.getTagByName(teamColor + "Target");
        if (tag == null) { return null; };

        return tag;
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
        teamColorBumpController();
        hoodGamepadController();
        canonGamepadController();
        intakeGamepadController();
        transferInputController();
        lowModeController();
    }


    private void lowModeController() {
        boolean pressed = gamepad1.y;

        if (pressed && !lastLowModeButton) {
            lowMode = !lowMode;
        }

        lastLowModeButton = pressed;
    }

    private void transferInputController()
    {
        if (gamepad1.dpad_up)
            transferController.lookUp();
        else
            transferController.lookDown();
    }

    private void intakeGamepadController() {
        if (gamepad1.left_trigger > 0.1) {
            intake = true;
            intakeMotor.setPower(0.8);
        } else {
            intake = false;
            intakeMotor.setPower(0);
        }

        if(gamepad1.x && !gamepad1.xWasPressed()){
            if (intakeReverse)
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            else
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            intakeReverse = !intakeReverse;
        }
    }

    private void canonGamepadController() {
        // Example mechanism control (range -1..1). Replace with your motor/CRServo as needed.
        cannonController.switchSpeed(gamepad1.right_trigger > 0.1);
        throwing = cannonController.canon.getVelocity() != 0;
    }

    private void hoodGamepadController() {
        if(gamepad1.a) {
            hoodControl.setAngleDeg(110);
        } else if(gamepad1.b) {
            hoodControl.setAngleDeg(-55);
        }
    }

    private void teamColorBumpController() {
        boolean pressed = gamepad1.right_bumper;

        if (pressed && !lastTeamColorButton) {
            teamColor = Objects.equals(teamColor, "Red") ? "Blue" : "Red";
        }

        lastTeamColorButton = pressed;
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
        telemetry.addData("Intake", intake);
        telemetry.addData("Ball Combination", ballCombination);
        telemetry.addData("Tower Distance", towerDistance);

        telemetry.update();
    }
}
