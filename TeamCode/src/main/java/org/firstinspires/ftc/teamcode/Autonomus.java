// Copyright Odysseas Chryssos All Rights Reserved
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Json Auto", group="Robot")

public class Autonomus extends LinearOpMode {

    // Declare component's variables
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor arm;
    private CRServo wristServo1;
    private CRServo wristServo2;
    private Servo claw;

    static final double APPROACH_SPEED  = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wristServo1 = hardwareMap.get(CRServo.class, "wristServo1");
        wristServo2 = hardwareMap.get(CRServo.class, "wristServo2");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Give update and wait for start
        telemetry.addData("Status", "Ready to collect speciments and park");
        waitForStart();

        // Set Starting Positions
        claw.setPosition(0.38);
        arm.setTargetPosition(700);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        // First back move for consistency

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(-1 * APPROACH_SPEED );
        backLeft.setPower(-1 * APPROACH_SPEED );
        backRight.setPower(-1 * APPROACH_SPEED );
        sleep(20);

        // first block swipe
        frontLeft.setPower(APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(APPROACH_SPEED);
        sleep(1720);

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(-1 * APPROACH_SPEED);
        sleep(500);

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(-1 * APPROACH_SPEED );
        backLeft.setPower(-1 * APPROACH_SPEED );
        backRight.setPower(-1 * APPROACH_SPEED );
        sleep(1720);

        // Second block swipe
        frontLeft.setPower(APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(APPROACH_SPEED);
        sleep(1800);

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(-1 * APPROACH_SPEED);
        sleep(580);

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(-1 * APPROACH_SPEED);
        backLeft.setPower(-1 * APPROACH_SPEED);
        backRight.setPower(-1 * APPROACH_SPEED);
        sleep(1740);

        // Third block swipe
        frontLeft.setPower(APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(APPROACH_SPEED);
        sleep(1700);

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(-1 * APPROACH_SPEED);
        sleep(770);

        frontLeft.setPower(-1 * APPROACH_SPEED);
        frontRight.setPower(-1 * APPROACH_SPEED);
        backLeft.setPower(-1 * APPROACH_SPEED);
        backRight.setPower(-1 * APPROACH_SPEED);
        sleep(1600);

        // Go to praking

        frontLeft.setPower(APPROACH_SPEED);
        frontRight.setPower(APPROACH_SPEED);
        backLeft.setPower(APPROACH_SPEED);
        backRight.setPower(APPROACH_SPEED);
        sleep(1700);

        frontLeft.setPower(APPROACH_SPEED);
        frontRight.setPower(-1 * APPROACH_SPEED);
        backLeft.setPower(-1 * APPROACH_SPEED);
        backRight.setPower(APPROACH_SPEED);
        sleep(1650);

        // Stop all motors

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        sleep(20000);
    }
}