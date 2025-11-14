package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Gyro-Test")
public class GyroTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private AnalogInput pot;
    // Overall drive scale (left bumper = slow, right bumper = fast)
    private double speedScale = 0.5;

    private boolean throwing;
    private double dispensePower;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        pot = hardwareMap.get(AnalogInput.class, "gyro");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Typical FTC setup: reverse the left side so +power drives all wheels forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        throwing = false;

        waitForStart();
        while (opModeIsActive()) {
            gamepadInput();
            mecanumDrive();
            doTelemetry();
        }
    }

    private double deadband(double v) {
        return Math.abs(v) > 0.05 ? v : 0.0;
    }

    private void mecanumDrive() {
        double y  = -deadband(gamepad1.left_stick_y);      // stick up = forward
        double x  = deadband(gamepad1.left_stick_x) * 1.1; // inverted strafe input
        double rx =  deadband(gamepad1.right_stick_x);

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
            speedScale = 0.4;  // precision
        } else if (gamepad1.right_bumper) {
            speedScale = 1.0;  // full speed
        } else {
            speedScale = 0.8;  // default
        }

        // Example mechanism control (range -1..1). Replace with your motor/CRServo as needed.
        if (gamepad1.right_trigger > 0.1) {
            throwing = true;
            dispensePower = gamepad1.right_trigger; // 0..1
        } else {
            throwing = false;
            dispensePower = 0.0;
        }
    }

    private void doTelemetry() {
        telemetry.addData("Speed Scale", "%.2f", speedScale);
        telemetry.addData("Throwing", throwing);
        telemetry.addData("Dispense Power", "%.2f", dispensePower);
        telemetry.update();
    }
}
