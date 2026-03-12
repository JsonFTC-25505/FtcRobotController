// COPYRIGHT ODYSSEAS CHRYSSOS | JsonFtc TALOS 2025-2026
//
// AprilTag Scanning System For the JsonFTC (TalOS) team.
// Recognises April Tags And Returns them into a "list"
//
// Methods:
// Name             | Arguments     | Action
// getTagById()     | int id        | Gets from the tags that have been detected with the specific id
// getTagByPrefix() | String prefix | Gets all the detected tags starting with the "prefix" string
// getTagByName()   | String name   | Gets all the detected tags with the specific name
//

package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

@Config
public class WebcamProcessor {
    private AprilTagProcessor aprilTagProcessor;
    private ColorBlobLocatorProcessor colorLocatorPurple;
    private ColorBlobLocatorProcessor colorLocatorGreen;

    private VisionPortal visionPortal;
    private ArrayList<AprilTagDetection> detectedTags = new ArrayList<>();
    private List<ColorBlobLocatorProcessor.Blob> detectedBlobs = new ArrayList<>();

    private Telemetry telemetry;

    // ===== Blob filter tuning =====
    // Tune these from Dashboard
    public static int MIN_BLOB_AREA = 1200;
    public static double MIN_BLOB_RADIUS = 18.0;
    public static double MIN_CIRCULARITY = 0.45;
    public static double MIN_DENSITY = 0.20;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                .build();

        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setBlurSize(5)
                .setErodeSize(15)
                .setDilateSize(15)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setBlurSize(5)
                .setErodeSize(15)
                .setDilateSize(15)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        builder.addProcessor(colorLocatorPurple);
        builder.addProcessor(colorLocatorGreen);

        visionPortal = builder.build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();

        // IMPORTANT: clear every frame so blobs do not accumulate forever
        detectedBlobs.clear();

        addValidBlobs(colorLocatorPurple.getBlobs());
        addValidBlobs(colorLocatorGreen.getBlobs());

        // Keep biggest blobs first
        detectedBlobs.sort((a, b) -> Integer.compare(b.getContourArea(), a.getContourArea()));
    }

    private void addValidBlobs(List<ColorBlobLocatorProcessor.Blob> blobs) {
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            if (isValidBallBlob(blob)) {
                detectedBlobs.add(blob);
            }
        }
    }

    private boolean isValidBallBlob(ColorBlobLocatorProcessor.Blob blob) {
        if (blob == null || blob.getCircle() == null) return false;

        double area = blob.getContourArea();
        double radius = blob.getCircle().getRadius();
        double circularity = blob.getCircularity();
        double density = blob.getDensity();

        if (area < MIN_BLOB_AREA) return false;
        if (radius < MIN_BLOB_RADIUS) return false;
        if (circularity < MIN_CIRCULARITY) return false;
        if (density < MIN_DENSITY) return false;

        return true;
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public List<ColorBlobLocatorProcessor.Blob> getDetectedBlobs() {
        return detectedBlobs;
    }

    public ColorBlobLocatorProcessor.Blob getBestBlob() {
        if (detectedBlobs.isEmpty()) return null;
        return detectedBlobs.get(0); // biggest valid blob
    }

    public boolean hasMetadata(AprilTagDetection aprilTagDetection) {
        return aprilTagDetection.metadata != null;
    }

    public void aprilTagTelemetry(AprilTagDetection detection) {
        if (detection == null) {
            return;
        }

        if (hasMetadata(detection)) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)",
                    detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                    detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)",
                    detection.center.x, detection.center.y));
        }
    }

    public void blobTelemetry(ColorBlobLocatorProcessor.Blob blob) {
        if (blob == null) {
            telemetry.addLine("\n==== No valid ball blob");
            return;
        }

        telemetry.addLine("\n==== Blob");
        telemetry.addLine(String.format("CRX %6.3f %6.1f %6.1f  (circ, rad px, x px)",
                blob.getCircularity(),
                blob.getCircle().getRadius(),
                blob.getCircle().getX()));
        telemetry.addLine(String.format("YAD %6.1f %d %6.3f  (y px, area px^2, density)",
                blob.getCircle().getY(),
                blob.getContourArea(),
                blob.getDensity()));
        telemetry.addData("Valid blobs", detectedBlobs.size());
    }

    public AprilTagDetection getTagById(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getTagByPrefix(String prefix) {
        for (AprilTagDetection detection : detectedTags) {
            if (hasMetadata(detection) && detection.metadata.name.startsWith(prefix)) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection getTagByName(String name) {
        for (AprilTagDetection detection : detectedTags) {
            if (hasMetadata(detection) && detection.metadata.name.equals(name)) {
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}