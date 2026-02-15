package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CoreHexUtil extends OpMode {
    private DcMotor dcMotor;

    @Override
    public void init() {
        dcMotor = hardwareMap.get(DcMotor.class, "coreHexTesterios");
        dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        dcMotor.setPower(1);
    }

    @Override
    public void loop() {
    }

    public void DoTelemetry(){
        telemetry.addData("DCMotor Power:", "%.2f", dcMotor.getPower());
    }
}
