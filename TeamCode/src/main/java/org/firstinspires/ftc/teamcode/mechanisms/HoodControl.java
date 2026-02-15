package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodControl {

    private Servo hoodServo;

    // Hood angle in degrees: -135 .. +135
    private float angleDeg = 0f;

    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        // Optional: start centered (0° -> 0.5)
        setAngleDeg(0f);
    }

    /** Set hood angle in degrees: -135 .. +135 */
    public void setAngleDeg(float angleDeg) {
        this.angleDeg = clamp(angleDeg, -135f, 135f);
        apply();
    }

    /** If you still want a 0..180 API, this remaps it to -135..+135 */
    public void setDegree(float deg0to180) {
        float d = clamp(deg0to180, 0f, 180f);
        float mapped = (d / 180f) * 270f - 135f; // 0->-135, 90->0, 180->135
        setAngleDeg(mapped);
    }

    private void apply() {
        double pos = angleToServoPos(angleDeg);
        hoodServo.setPosition(pos);
        doTelemetry(angleDeg, pos);
    }

    private static double angleToServoPos(float angleDeg) {
        return (angleDeg + 135.0) / 270.0; // result in [0..1]
    }

    private static float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void doTelemetry(double targetAngleDeg, double targetPos) {
        telemetry.addLine(String.format("Hood Angle: %.2f°", targetAngleDeg));
        telemetry.addLine(String.format("Target pos: %.3f", targetPos));
        telemetry.addLine(String.format("Servo pos:  %.3f", hoodServo.getPosition()));
    }
}
