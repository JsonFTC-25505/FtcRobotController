package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.MainTeleOP;

import java.util.LinkedList;

public class Sorter {

    ColorSensor colorSensor = new ColorSensor();
    Trajectory trajectory = new Trajectory();

    CRServo turner;

    LinkedList<String> ballsList = new LinkedList<>();
    LinkedList<String> sort = new LinkedList<>();

    ColorSensor.DetectedColor currentColor;

    // manual override
    private boolean manualOverride = false;

    public void init(HardwareMap hardwareMap){
        colorSensor.init(hardwareMap);
        turner = hardwareMap.get(CRServo.class, "sorterServo");
    }

    public void loop(Telemetry telemetry) {
        // If driver is manually controlling the sorter, do not run auto logic
//        if (manualOverride) {
//            return;
//        }

        ColorSensor.DetectedColor detectedColor = colorSensor.getDetectedColor();

        telemetry.addLine(String.valueOf(detectedColor));
        if (detectedColor != ColorSensor.DetectedColor.UNKNOWN &&
                detectedColor != ColorSensor.DetectedColor.B && detectedColor != ColorSensor.DetectedColor.W) {

            ballsList.add(detectedColor.toString());

            while (detectedColor != ColorSensor.DetectedColor.W) {
                turner.setPower(1);   // one direction for indexing
                detectedColor = colorSensor.getDetectedColor();
            }

            turner.setPower(0);
        }
    }

    /**
     * Manual driver control for the sorter.
     * right = clockwise
     * left  = counter-clockwise
     */
    public void manualControl(boolean right, boolean left) {
//        if (right && !left) {
//            manualOverride = true;
//            turner.setPower(1.0);
//        } else if (left && !right) {
//            manualOverride = true;
//            turner.setPower(-1.0);
//        } else {
//            manualOverride = false;
//            turner.setPower(0.0);
//        }
    }

    public void stop() {
        manualOverride = false;
        turner.setPower(0.0);
    }

    public void shoot(MainTeleOP.Balls balls){
        if (balls == null || ballsList.isEmpty()) return;

        sort.clear();
        for (char ch : balls.toString().toCharArray()) {
            sort.add(String.valueOf(ch));
        }

        // probably what you intended:
        while (!ballsList.equals(sort)) {
            turner.setPower(1.0);

            String lastItem = ballsList.get(2);
            ballsList.remove(2);
            ballsList.add(0, lastItem);

            // IMPORTANT:
            // in real robot code you should detect one slot movement with sensor/encoder
            // then stop briefly, otherwise this loop will spin too fast in software
            break;
        }

        turner.setPower(0.0);

        trajectory.compute(0); // need real numbers
        currentColor = colorSensor.getDetectedColor();

        while (currentColor != ColorSensor.DetectedColor.W) {
            turner.setPower(1.0);
            currentColor = colorSensor.getDetectedColor();
        }

        turner.setPower(0.0);
    }
}