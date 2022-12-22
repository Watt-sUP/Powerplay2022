package org.firstinspires.ftc.teamcode.autonom.apriltag;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "OpenCV Test", group = "Testing")
public class AutonomousInit extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // NOTE: Ignore the stuff below, it's used for 3D tracking which isn't necessary
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int LEFT = 0, MID = 1, RIGHT = 2;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("An error occurred while opening camera\nError code: " + errorCode);
                telemetry.update();
                return;
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MID || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("A tag has been found.\nPosition:" + getPosition(tagOfInterest.id));
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("No tag of interest has been found.");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("Last known position: Unknown");
                    }
                    else
                    {
                        telemetry.addLine("Last known position:" + getPosition(tagOfInterest.id));
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("No tag has been found.");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("Last known position: Unknown");
                }
                else
                {
                    telemetry.addLine("Last known position:" + getPosition(tagOfInterest.id));
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addData("Detected tag type", getPosition(tagOfInterest.id));
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag sighted during the initialization loop");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            sleep(20 * 1000);
        }
        else
        {
            // TODO: Insert some actual code
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }

    String getPosition(int id)
    {
        if (id == LEFT)
            return "Left";
        else if (id == RIGHT)
            return "Right";
        else if (id == MID)
            return "Middle";

        return "Unknown";
    }
}
