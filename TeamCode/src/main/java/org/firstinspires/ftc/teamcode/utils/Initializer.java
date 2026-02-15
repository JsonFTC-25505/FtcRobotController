package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivers.MPU6050;

public class Initializer {

    // TO-DO :3
//    private void initializeTelemetry() {
//        telemetry = new MultipleTelemetry(
//                telemetry,
//                FtcDashboard.getInstance().getTelemetry()
//        );
//    }
//
//    private void initializeMotors(){
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        throwing = false;
//        intake = false;
//    }
//
//    private void initializeSensors(){
//        // Get our MPU device from the hardwareMap (name must match RC config)
//        imu = hardwareMap.get(MPU6050.class, "imu");
//        imu.initialize();
//
//        timer.reset();
//        calibrateGyroZ();
//        aprilTagWebcam.init(hardwareMap, telemetry);
//    }
//
//    private void initializeControllers(){
//        hoodControl.init(hardwareMap, telemetry);
//        cannonController.init(hardwareMap);
//    }
}
