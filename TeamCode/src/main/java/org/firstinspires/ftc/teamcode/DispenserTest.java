package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Dispensing Test Code")
public class DispenserTest extends LinearOpMode {

    private DcMotor dispenser;

    boolean throwing;
    float dispensePower;

    /**
     * This function is executed when this Op Mode is selected.
     */
    @Override
    public void runOpMode() {

        dispenser = hardwareMap.get(DcMotor.class, "dispenser");

        throwing = false;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                GAMEPAD_INPUT();
                TELEMETRY();
                dispenser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dispenser.setPower(dispensePower);
            }
        }
    }

    private void GAMEPAD_INPUT() {
        if (gamepad1.right_trigger > 0.1) {
            throwing = true;
            dispensePower = (255); // * gamepad1.right_trigger)
        }else{
            throwing = false;
            dispensePower = 0;

        }
    }

    /**
     * Telemetry Data display!
     */
    private void TELEMETRY() {
        telemetry.addData("Throwing? :", throwing);
        telemetry.addData("Power: ", dispensePower);
        telemetry.update();
    }
}