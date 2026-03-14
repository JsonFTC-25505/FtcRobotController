package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.utils.RGBBundle;

@Config
public class ColorSensor {

    private NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        G,
        P,
        B, // Black
        W, // White
        UNKNOWN
    }

    // =========================
    // Calibrated reference colors
    // =========================
    // These should be measured on the real robot with the real distance
    public static double PR = 0.140;
    public static double PG = 0.164;
    public static double PB = 0.265;

    public static double GR = 0.065;
    public static double GG = 0.245;
    public static double GB = 0.179;

    // =========================
    // Sensor settings
    // =========================
    public static double SENSOR_GAIN = 8.0;

    // =========================
    // Thresholds
    // =========================

    // Minimum alpha needed before dividing by alpha for chromatic checks
    public static double MIN_ALPHA_FOR_CHROMA = 0.01;

    // Very dark + extremely low alpha = probably no meaningful target under sensor
    public static double NO_TARGET_MAX_BRIGHTNESS = 0.025;
    public static double NO_TARGET_MAX_ALPHA = 0.010;

    // Black should be dark, but not "nothing"
    public static double BLACK_MAX_BRIGHTNESS = 0.090;
    public static double BLACK_MIN_ALPHA = 0.012;
    public static double BLACK_MAX_ALPHA = 0.080;

    // White should be bright and fairly balanced across channels
    public static double WHITE_MIN_BRIGHTNESS = 0.180;
    public static double WHITE_MIN_ALPHA = 0.120;
    public static double WHITE_MAX_CHANNEL_SPREAD = 0.080;

    // Max squared distance allowed for a color match
    // Lower = stricter detection
    public static double COLOR_MAX_DISTANCE = 0.0035;

    // =========================
    // Stability filter
    // =========================
    // Number of same raw detections needed before output changes
    public static int REQUIRED_CONSECUTIVE_READS = 2;

    private DetectedColor lastRawColor = DetectedColor.UNKNOWN;
    private DetectedColor stableColor = DetectedColor.UNKNOWN;
    private int sameColorCount = 0;

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorS");
        colorSensor.setGain((float) SENSOR_GAIN);
    }

    /**
     * Stable color detection.
     * Use this in robot logic.
     */
    public DetectedColor getDetectedColor() {
        DetectedColor raw = getRawDetectedColor();

        if (raw == lastRawColor) {
            sameColorCount++;
        } else {
            lastRawColor = raw;
            sameColorCount = 1;
        }

        if (sameColorCount >= REQUIRED_CONSECUTIVE_READS) {
            stableColor = raw;
        }

        return stableColor;
    }

    /**
     * Immediate color detection without stability filtering.
     * Useful for debugging / telemetry.
     */
    public DetectedColor getRawDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float alpha = colors.alpha;
        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        // Raw brightness is better for black / white / no-target decisions
        float brightness = red + green + blue;

        // -------------------------
        // 0) NO TARGET / GAP / AIR
        // -------------------------
        // If both brightness and alpha are extremely low, it is probably not a real surface.
        if (brightness <= NO_TARGET_MAX_BRIGHTNESS && alpha <= NO_TARGET_MAX_ALPHA) {
            return DetectedColor.UNKNOWN;
        }

        // -------------------------
        // 1) BLACK detection
        // -------------------------
        // Black is dark, but should still return slightly more signal than "nothing".
        if (brightness <= BLACK_MAX_BRIGHTNESS
                && alpha >= BLACK_MIN_ALPHA
                && alpha <= BLACK_MAX_ALPHA) {
            return DetectedColor.B;
        }

        // From here on, we need enough alpha to safely normalize RGB by alpha.
        if (alpha <= MIN_ALPHA_FOR_CHROMA) {
            return DetectedColor.UNKNOWN;
        }

        // Normalized against alpha -> better for hue/chromatic comparison
        float normR = red / alpha;
        float normG = green / alpha;
        float normB = blue / alpha;

        // -------------------------
        // 2) WHITE detection
        // -------------------------
        // White should be bright and the channels should be similar
        float maxChannel = Math.max(normR, Math.max(normG, normB));
        float minChannel = Math.min(normR, Math.min(normG, normB));
        float spread = maxChannel - minChannel;

        if (brightness >= WHITE_MIN_BRIGHTNESS
                && alpha >= WHITE_MIN_ALPHA
                && spread <= WHITE_MAX_CHANNEL_SPREAD) {
            return DetectedColor.W;
        }

        // -------------------------
        // 3) Purple / Green detection
        // -------------------------
        double distPurple = squaredDistance(normR, normG, normB, PR, PG, PB);
        double distGreen  = squaredDistance(normR, normG, normB, GR, GG, GB);

        boolean purpleValid = distPurple <= COLOR_MAX_DISTANCE;
        boolean greenValid  = distGreen <= COLOR_MAX_DISTANCE;

        if (purpleValid && greenValid) {
            return (distPurple < distGreen) ? DetectedColor.P : DetectedColor.G;
        } else if (purpleValid) {
            return DetectedColor.P;
        } else if (greenValid) {
            return DetectedColor.G;
        }

        return DetectedColor.UNKNOWN;
    }

    /**
     * Returns current normalized RGB values (r/alpha, g/alpha, b/alpha).
     * Good for telemetry / calibration.
     */
    public RGBBundle getCurrentColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.alpha <= MIN_ALPHA_FOR_CHROMA) {
            return new RGBBundle(0, 0, 0);
        }

        float normR = colors.red / colors.alpha;
        float normG = colors.green / colors.alpha;
        float normB = colors.blue / colors.alpha;

        return new RGBBundle(normR, normG, normB);
    }

    /**
     * Optional raw brightness helper for telemetry.
     */
    public double getBrightness() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.red + colors.green + colors.blue;
    }

    /**
     * Optional alpha helper for telemetry.
     */
    public double getAlpha() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        return colors.alpha;
    }

    /**
     * Optional raw RGBA helper if needed elsewhere.
     */
    public NormalizedRGBA getRawColors() {
        return colorSensor.getNormalizedColors();
    }

    private double squaredDistance(double r1, double g1, double b1,
                                   double r2, double g2, double b2) {
        double dr = r1 - r2;
        double dg = g1 - g2;
        double db = b1 - b2;
        return dr * dr + dg * dg + db * db;
    }

    /**
     * Resets the stability filter.
     * Useful when starting a new cycle if you want fresh state.
     */
    public void resetDetectionState() {
        lastRawColor = DetectedColor.UNKNOWN;
        stableColor = DetectedColor.UNKNOWN;
        sameColorCount = 0;
    }
}