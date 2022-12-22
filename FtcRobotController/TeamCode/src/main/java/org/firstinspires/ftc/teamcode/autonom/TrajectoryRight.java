package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonom.apriltag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Glisiere;
import org.firstinspires.ftc.teamcode.hardware.Turela;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "Traiectorie Dreapta", group = "auto")
public class TrajectoryRight extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // NOTE: Ignore the stuff below, it's used for 3D tracking which isn't necessary
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    public static int FORWARD = 50;
    public static int STRAFE = 24;
    public static int SPEED_LIMIT = 45, TURN_LIMIT = 4;
    public static double DEGET_TIME = 6.5, FORWARD_OFFSET = 10, BACK_OFFSET = 12, PARK_DIST = 25, GLIS_OFFSET = 1.5, TURN = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("An error occurred while opening camera\nError code: " + errorCode);
                telemetry.update();
                return;
            }
        });

        telemetry.setMsTransmissionInterval(50);

        Colectare deget = new Colectare(hardwareMap);
        Glisiere glis = new Glisiere(hardwareMap);
        Turela tur = new Turela(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections)
                    if (tag.id == 0 || tag.id == 1 || tag.id == 2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }

                if (tagFound)
                    telemetry.addLine("A tag has been found.\nPosition:" + getPosition(tagOfInterest.id));
                else {
                    telemetry.addLine("No tag of interest has been found.");

                    if (tagOfInterest == null)
                        telemetry.addLine("Last known position: Unknown");
                    else
                        telemetry.addLine("Last known position:" + getPosition(tagOfInterest.id));
                }

            } else {
                telemetry.addLine("No tag has been found.");
                if (tagOfInterest == null)
                    telemetry.addLine("Last known position: Unknown");
                else
                    telemetry.addLine("Last known position:" + getPosition(tagOfInterest.id));
            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest.id == 0)
            PARK_DIST = 2;
        else if (tagOfInterest.id == 1)
            PARK_DIST = 25;
        else if (tagOfInterest.id == 2)
            PARK_DIST = 47;
        else return;

        TrajectorySequence testTraj = drive.trajectorySequenceBuilder(new Pose2d())
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(SPEED_LIMIT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SPEED_LIMIT)
                )
                .setTurnConstraint(TURN_LIMIT, TURN_LIMIT)
                .forward(FORWARD)
                .addTemporalMarker(0.2, 0, () -> {
                    glis.setToPosition(5);
                })
                .addTemporalMarker(DEGET_TIME, () -> {
                    deget.toggleDeget();
                })
                .turn(Math.toRadians(TURN))
                .forward(STRAFE)
                .strafeLeft(BACK_OFFSET)
                .waitSeconds(1.5)
                .strafeRight(FORWARD_OFFSET)
                .addDisplacementMarker(() -> {
                    deget.toggleDeget();
                    tur.setToTicks(0);
                })
                .back(PARK_DIST)
                .addTemporalMarker(DEGET_TIME + GLIS_OFFSET, () -> {
                    glis.setToTicks(0);
                })
                .resetConstraints()
                .resetTurnConstraint()
                .build();

        sleep(400);
        glis.setToPosition(1);
        drive.followTrajectorySequence(testTraj);
    }

    String getPosition(int id)
    {
        if (id == 0)
            return "Left";
        else if (id == 1)
            return "Middle";
        else if (id == 2)
            return "Right";

        return "Unknown";
    }
}