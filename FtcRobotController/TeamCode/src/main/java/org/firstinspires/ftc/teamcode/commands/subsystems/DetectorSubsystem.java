package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.autonom.apriltag.AprilTagDetector;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Map;

/**
 * Command-based subsystem for the AprilTag detector.
 */
public class DetectorSubsystem extends SubsystemBase {

    private final AprilTagDetector detector;
    public int lastDetection = -1;

    /**
     * Creates a new instance of the subsystem.
     * @param hardwareMap HardwareMap object, used to initialize the detector
     * @param targets List of target IDs to detect
     */
    public DetectorSubsystem(HardwareMap hardwareMap, Integer... targets) {
        detector = new AprilTagDetector(hardwareMap);
        detector.WIDTH = 1280;
        detector.HEIGHT = 720;
        detector.ORIENTATION = OpenCvCameraRotation.SIDEWAYS_LEFT;
        detector.GPU_ENABLED = true;

        detector.init();
        detector.setTargets(targets);
    }

    /**
     * Overrides the current target IDs.
     * @param targets List of target IDs to detect
     */
    public void setTargets(Integer... targets) {
        detector.setTargets(targets);
    }

    /**
     * Gets the current detection. If a tag is detected, the ID of the tag is stored in lastDetection.
     * @return A map containing the ID of the detected tag and its coordinates in the image.
     */
    public Map<String, Integer> getDetection() {
        Map<String, Integer> detection = detector.getDetection();
        if (detection != null) {
            lastDetection = detection.get("id");
        }

        return detection;
    }

    /**
     * Closes the detector to free up resources.
     */
    public void close() {
        detector.closeAsync();
    }
}