package org.firstinspires.ftc.teamcode.autonom.apriltag;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTag2dPipeline extends OpenCvPipeline {
    private long nativeAprilTagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagSize = 0.166;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public AprilTag2dPipeline() {
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    protected void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeAprilTagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
            nativeAprilTagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeAprilTagPtr was NULL");
        }
    }

    /**
     * <p>Processes frames received from the camera, runs detections, and draws a square on the Driver Station around the detected tag.</p>
     * <p>Doesn't need to be called by the user.</p>
     *
     * @param input The frame from the camera
     * @return The modified frame
     */
    @Override
    public Mat processFrame(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeAprilTagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, grey, tagSize, fx, fy, cx, cy);
        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        for (AprilTagDetection detection : detections) {
            draw2dSquare(input, detection.corners);
        }

        return input;
    }

    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    /**
     * Gets the latest detections from the pipeline.
     *
     * @return An ArrayList of AprilTagDetection objects
     */
    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync) {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }


    /**
     * Draws a square on the image
     *
     * @param buf    The image to draw on
     * @param points The corners of the square
     */
    void draw2dSquare(Mat buf, Point[] points) {
        Scalar blue = new Scalar(7, 197, 235, 255);

        Imgproc.line(buf, points[0], points[1], blue);
        Imgproc.line(buf, points[1], points[2], blue);
        Imgproc.line(buf, points[2], points[3], blue);
        Imgproc.line(buf, points[3], points[0], blue);
    }
}