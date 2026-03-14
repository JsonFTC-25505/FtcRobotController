package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.MainTeleOP;

import java.util.LinkedList;
import java.util.Objects;

public class Sorter {

    private final ColorSensor colorSensor = new ColorSensor();
    private Servo turner;

    // Stored detected artifact order
    private final LinkedList<String> ballsList = new LinkedList<>();
    private final LinkedList<String> sort = new LinkedList<>();

    private ColorSensor.DetectedColor currentColor = ColorSensor.DetectedColor.UNKNOWN;

    // =========================
    // Manual override
    // =========================
    private boolean manualOverride = false;
    private boolean lastSwitchButton = false;

    // =========================
    // Non-blocking auto actions
    // =========================
    private boolean steppingToWhite = false;
    private boolean shootingSequenceActive = false;

    private enum BlackSeekMode {
        NONE,
        TO_BLACK_NOW,
        TO_NEXT_BLACK
    }

    private BlackSeekMode blackSeekMode = BlackSeekMode.NONE;

    // =========================
    // Tunables
    // =========================
    private static final double AUTO_FEED_POWER = 0.45;
    private static final double STEP_TO_WHITE_POWER = 0.10;
    private static final double MANUAL_POWER = 1.0;
    private static final double ROTATE_TO_BLACK_POWER = 0.20;

    private int whiteSeenCount = 0;
    private static final int REQUIRED_WHITE_READS = 1;
    private boolean wasOffWhiteBeforeStep = false;

    private int blackSeenCount = 0;
    private static final int REQUIRED_BLACK_READS = 3;

    private boolean wasOffBlackBeforeSeek = false;
    private int nonBlackSeenCount = 0;
    private static final int REQUIRED_OFF_BLACK_READS = 2;

    // After leaving the starting black, require some travel before black can stop again
    private int travelAfterLeavingBlackCount = 0;
    private static final int MIN_TRAVEL_BEFORE_ACCEPT_BLACK = 8;

    // Normal auto-feed helper for G/P -> UNKNOWN transitions
    private boolean recentlySawArtifact = false;
    private int unknownGraceCount = 0;
    private static final int UNKNOWN_GRACE_READS = 4;

    private double intakePos1 = 0.05;
    private double intakePos2 = 1;

    private double shootPos1 = 0.275;
    private double shootPos2 = 0.753;


    boolean lastClicked = false;
    boolean lastClicked2 = false;

    private boolean LastBall = false;

    public void init(HardwareMap hardwareMap) {
        colorSensor.init(hardwareMap);
        turner = hardwareMap.get(Servo.class, "sorterServo");
        turner.setPosition(intakePos1);
    }

    public void loop(Telemetry telemetry, Gamepad gamepad) {
        // Manual mode wins over everything
        if (manualOverride) {
            telemetry.addLine("Sorter: MANUAL");
            telemetry.addData("Detected Color", colorSensor.getDetectedColor());
            telemetry.addData("Stepping To White", steppingToWhite);
            telemetry.addData("Shooting Sequence", shootingSequenceActive);
            telemetry.addData("Black Seek Mode", blackSeekMode);
            return;
        }
        ColorSensor.DetectedColor detectedColor = colorSensor.getDetectedColor();

        telemetry.addLine(String.valueOf(detectedColor));

        boolean dPadLeft = gamepad.dpad_left;
        boolean dPadRight = gamepad.dpad_right;
        if (dPadLeft && !lastClicked){

            telemetry.addLine("detected");
            if (LastBall){
                turner.setPosition(intakePos1);
                LastBall = !LastBall;
            }else{
                turner.setPosition(intakePos2);
                LastBall = !LastBall;
            }
        }

        if (dPadRight && !lastClicked2){
            if (LastBall){
                turner.setPosition(shootPos1);
            }else{
                turner.setPosition(shootPos2);
            }
            LastBall = !LastBall;
        }

        lastClicked = dPadLeft;
        lastClicked2 = dPadRight;


    }


    /**
     * Manual driver control for the sorter.
     * right = clockwise
     * left  = counter-clockwise
     * sw    = toggle manual override
     */
    public void manualControl(boolean right, boolean left, boolean sw) {
        if (sw && !lastSwitchButton) {
            manualOverride = !manualOverride;

            if (manualOverride) {
                steppingToWhite = false;
                shootingSequenceActive = false;
                blackSeekMode = BlackSeekMode.NONE;

                recentlySawArtifact = false;
                unknownGraceCount = 0;

                blackSeenCount = 0;
                nonBlackSeenCount = 0;
                travelAfterLeavingBlackCount = 0;
                wasOffBlackBeforeSeek = false;

                //turner.setPower(0.0);
            }
        }
        lastSwitchButton = sw;

        if (manualOverride) {
            if (right && !left) {
                //turner.setPower(MANUAL_POWER);
            } else if (left && !right) {
                //turner.setPower(-MANUAL_POWER);
            } else {
                //turner.setPower(0.0);
            }
        }
    }

    public void stop() {
        manualOverride = false;
        steppingToWhite = false;
        shootingSequenceActive = false;
        blackSeekMode = BlackSeekMode.NONE;
        lastSwitchButton = false;

        recentlySawArtifact = false;
        unknownGraceCount = 0;

        blackSeenCount = 0;
        nonBlackSeenCount = 0;
        travelAfterLeavingBlackCount = 0;
        wasOffBlackBeforeSeek = false;

        //turner.setPower(0.0);
    }

    /**
     * Starts a non-blocking step-until-white action.
     */
    public void startStepToWhite() {
        steppingToWhite = true;
        whiteSeenCount = 0;
        wasOffWhiteBeforeStep = false;
        colorSensor.resetDetectionState();
    }

    /**
     * Compatibility wrapper if older code still calls doStepToWhite().
     */
    public void doStepToWhite() {
        startStepToWhite();
    }

    /**
     * Non-blocking white step updater.
     * Returns true when white is reached.
     */
    private boolean updateStepToWhite(Telemetry telemetry) {
        ColorSensor.DetectedColor detectedColor = colorSensor.getDetectedColor();

        telemetry.addData("StepToWhite Color", detectedColor);
        telemetry.addData("White Seen Count", whiteSeenCount);
        telemetry.addData("Was Off White", wasOffWhiteBeforeStep);

        // First make sure we have actually left white
        if (!wasOffWhiteBeforeStep) {
            if (detectedColor != ColorSensor.DetectedColor.W) {
                wasOffWhiteBeforeStep = true;
            }

            //turner.setPower(STEP_TO_WHITE_POWER);
            return false;
        }

        // After leaving white, wait until we re-enter white consistently
        if (detectedColor == ColorSensor.DetectedColor.W) {
            whiteSeenCount++;
        } else {
            whiteSeenCount = 0;
        }

        if (whiteSeenCount >= REQUIRED_WHITE_READS) {
            //turner.setPower(0.0);
            steppingToWhite = false;
            return true;
        }

        //turner.setPower(STEP_TO_WHITE_POWER);
        return false;
    }

    /**
     * Rotate clockwise until black is found.
     * If already on black, it may stop almost immediately.
     */
    public void startRotateToBlack() {
        blackSeekMode = BlackSeekMode.TO_BLACK_NOW;

        blackSeenCount = 0;
        nonBlackSeenCount = 0;
        travelAfterLeavingBlackCount = 0;
        wasOffBlackBeforeSeek = false;

        recentlySawArtifact = false;
        unknownGraceCount = 0;

        colorSensor.resetDetectionState();
    }

    /**
     * Rotate clockwise until the NEXT black marker is found.
     * Leaves the current black first, then ignores early black glitches,
     * then only stops on a valid later black.
     */
    public void startRotateToNextBlack() {
        blackSeekMode = BlackSeekMode.TO_NEXT_BLACK;

        blackSeenCount = 0;
        nonBlackSeenCount = 0;
        travelAfterLeavingBlackCount = 0;
        wasOffBlackBeforeSeek = false;

        recentlySawArtifact = false;
        unknownGraceCount = 0;

        colorSensor.resetDetectionState();
    }

    private void updateBlackSeek(Telemetry telemetry) {
        ColorSensor.DetectedColor detectedColor = colorSensor.getDetectedColor();
        ColorSensor.DetectedColor rawColor = colorSensor.getRawDetectedColor();

        telemetry.addLine("Sorter: SEEKING BLACK");
        telemetry.addData("Black Seek Mode", blackSeekMode);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Raw Color", rawColor);
        telemetry.addData("Black Seen Count", blackSeenCount);
        telemetry.addData("Non-Black Seen Count", nonBlackSeenCount);
        telemetry.addData("Travel After Leaving Black", travelAfterLeavingBlackCount);
        telemetry.addData("Was Off Black", wasOffBlackBeforeSeek);

        switch (blackSeekMode) {
            case TO_BLACK_NOW:
                // require multiple consecutive B readings before stopping
                if (detectedColor == ColorSensor.DetectedColor.B) {
                    blackSeenCount++;
                } else {
                    blackSeenCount = 0;
                }

                if (blackSeenCount >= REQUIRED_BLACK_READS) {
                    //turner.setPower(0.0);
                    blackSeekMode = BlackSeekMode.NONE;
                    return;
                }

                //turner.setPower(ROTATE_TO_BLACK_POWER);
                return;

            case TO_NEXT_BLACK:
                // First make sure we have really left the starting black
                if (!wasOffBlackBeforeSeek) {
                    if (detectedColor != ColorSensor.DetectedColor.B) {
                        nonBlackSeenCount++;
                    } else {
                        nonBlackSeenCount = 0;
                    }

                    if (nonBlackSeenCount >= REQUIRED_OFF_BLACK_READS) {
                        wasOffBlackBeforeSeek = true;
                        blackSeenCount = 0;
                        travelAfterLeavingBlackCount = 0;
                    }

                    //turner.setPower(ROTATE_TO_BLACK_POWER);
                    return;
                }

                // We have left black. Keep counting travel while we are not on black.
                if (detectedColor != ColorSensor.DetectedColor.B) {
                    travelAfterLeavingBlackCount++;
                    blackSeenCount = 0;
                    //turner.setPower(ROTATE_TO_BLACK_POWER);
                    return;
                }

                // Ignore any black seen too early after leaving the first black marker
                if (travelAfterLeavingBlackCount < MIN_TRAVEL_BEFORE_ACCEPT_BLACK) {
                    blackSeenCount = 0;
                    //turner.setPower(ROTATE_TO_BLACK_POWER);
                    return;
                }

                // Now we are far enough away: require multiple B reads before stopping
                blackSeenCount++;

                if (blackSeenCount >= REQUIRED_BLACK_READS) {
                    //turner.setPower(0.0);
                    blackSeekMode = BlackSeekMode.NONE;
                    return;
                }

                //turner.setPower(ROTATE_TO_BLACK_POWER);
                return;

            case NONE:
            default:
                //turner.setPower(0.0);
        }
    }

    /**
     * Starts a non-blocking shooting/indexing sequence.
     * This version only manages sorter indexing state.
     * It does NOT directly control cannon/transfer hardware.
     */
    public void shoot(MainTeleOP.Balls balls) {
        if (balls == null || ballsList.isEmpty()) return;
        if (shootingSequenceActive) return;

        sort.clear();
        for (char ch : balls.toString().toCharArray()) {
            sort.add(String.valueOf(ch));
        }

        if (sort.isEmpty()) return;

        shootingSequenceActive = true;
        startStepToWhite();
    }

    private void updateShootingSequence(Telemetry telemetry) {
        telemetry.addLine("Sorter: SHOOTING");
        telemetry.addData("BallsList", ballsList.toString());
        telemetry.addData("Sort", sort.toString());

        // End conditions
        if (sort.isEmpty() || ballsList.isEmpty()) {
            shootingSequenceActive = false;
            steppingToWhite = false;
            blackSeekMode = BlackSeekMode.NONE;
            //turner.setPower(0.0);
            return;
        }

        // First align to white
        if (steppingToWhite) {
            boolean done = updateStepToWhite(telemetry);
            if (!done) return;
        }

        String wanted = sort.getFirst();
        String currentIndexedBall = getIndexedBall();

        telemetry.addData("Wanted Ball", wanted);
        telemetry.addData("Indexed Ball", currentIndexedBall);

        if (currentIndexedBall == null) {
            shootingSequenceActive = false;
            //turner.setPower(0.0);
            return;
        }

        if (Objects.equals(currentIndexedBall, wanted)) {
            removeIndexedBall();
            sort.removeFirst();

            if (sort.isEmpty() || ballsList.isEmpty()) {
                shootingSequenceActive = false;
                //turner.setPower(0.0);
                return;
            }
        }

        // Move to next slot
        startStepToWhite();
    }

    /**
     * Ball currently considered at the indexed shoot position.
     */
    private String getIndexedBall() {
        if (ballsList.size() > 1) {
            return ballsList.get(1);
        } else if (!ballsList.isEmpty()) {
            return ballsList.getFirst();
        }
        return null;
    }

    private void removeIndexedBall() {
        if (ballsList.size() > 1) {
            ballsList.remove(1);
        } else if (!ballsList.isEmpty()) {
            ballsList.removeFirst();
        }
    }

    public boolean isManualOverride() {
        return manualOverride;
    }

    public boolean isSteppingToWhite() {
        return steppingToWhite;
    }

    public boolean isShootingSequenceActive() {
        return shootingSequenceActive;
    }

    public boolean isSeekingBlack() {
        return blackSeekMode != BlackSeekMode.NONE;
    }

    public ColorSensor.DetectedColor getCurrentDetectedColor() {
        return currentColor;
    }

    public LinkedList<String> getBallsList() {
        return ballsList;
    }

    public void clearBallsList() {
        ballsList.clear();
    }

    public void addBall(String color) {
        ballsList.add(color);
    }
}