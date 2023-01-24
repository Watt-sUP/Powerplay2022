package org.firstinspires.ftc.teamcode.autonom.apriltag;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.text.DecimalFormat;
import java.util.List;
import java.util.Map;

@TeleOp(name = "OpenCV FTCLib Test", group = "Test")
public class OpenCVTest extends LinearOpMode {

    AprilTagDetector aprilTagDetector;
    OpenCvCamera camera;
    List<Integer> targets;

    @Override
    public void runOpMode() throws InterruptedException {

        DecimalFormat df = new DecimalFormat("0.00");
        aprilTagDetector = new AprilTagDetector(hardwareMap);

        aprilTagDetector.WIDTH = 1280;
        aprilTagDetector.HEIGHT = 720;
        aprilTagDetector.ORIENTATION = OpenCvCameraRotation.SIDEWAYS_LEFT;
        aprilTagDetector.GPU_ENABLED = true;

        aprilTagDetector.init();
        aprilTagDetector.setTargets(0, 1, 2);
        camera = aprilTagDetector.getCamera();

        telemetry.setAutoClear(true);
        waitForStart();

        while (opModeIsActive()) {
            Map<String, Integer> detection = aprilTagDetector.getDetection();
            targets = aprilTagDetector.getTargets();

            telemetry.addLine("Camera FPS: " + df.format(camera.getFps()));
            telemetry.addLine("Max theoretical FPS: " + df.format(camera.getCurrentPipelineMaxFps()));
            telemetry.addLine("Current targets: " + (targets != null ? targets.toString() : "None"));

            if (detection != null) {
                telemetry.addLine("Detection ID: " + detection.get("id"));
                telemetry.addLine("Detection X: " + detection.get("x"));
                telemetry.addLine("Detection Y: " + detection.get("y"));
            }

            telemetry.update();
        }
        aprilTagDetector.close();
    }
}
