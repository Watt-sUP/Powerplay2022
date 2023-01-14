package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonom.apriltag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;
import org.firstinspires.ftc.teamcode.hardware.Glisiere;
import org.firstinspires.ftc.teamcode.hardware.Turela;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@Config
@Autonomous(name = "Autonom Stanga Bun", group = "auto")
public class AutonomStangaBun extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // NOTE: Ignore the stuff below, it's used for 3D tracking which isn't necessary
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagSize = 0.166;

    AprilTagDetection tagOfInterest = null;

    public static double STRAFE = 5, FORWARD = 50, PARK_DIST = 24;
    public static int ROTATION = 0;
    public static double TURELA_TICKS = -505;
    public static double SPEED_LIMIT = 40;

    public static Preload PRELOAD = new Preload(0.25, 0.74, 1.25);
    public static Cycle CYCLE = new Cycle(0.33, 0.96, 960, 450, 70, 1.9);

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(90));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

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
            }
        });

        Colectare colectare = new Colectare(hardwareMap);
        Glisiere glis = new Glisiere(hardwareMap);
        Turela turela = new Turela(hardwareMap);
        Foarfeca foarfeca = new Foarfeca(hardwareMap);

        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                    if(tag.id == 0 || tag.id == 1 || tag.id == 2)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }

                if(tagFound)
                    telemetry.addLine("A tag has been found.\nPosition:" + getPosition(tagOfInterest.id));
                else
                {
                    telemetry.addLine("No tag of interest has been found.");

                    if(tagOfInterest == null)
                        telemetry.addLine("Last known position: Unknown");
                    else
                        telemetry.addLine("Last known position:" + getPosition(tagOfInterest.id));
                }

            }
            else
            {
                telemetry.addLine("No tag has been found.");
                if(tagOfInterest == null)
                    telemetry.addLine("Last known position: Unknown");
                else
                    telemetry.addLine("Last known position:" + getPosition(tagOfInterest.id));
            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest.id == 0) {
            PARK_DIST = 16;
            ROTATION = 90;
        }
        else if (tagOfInterest.id == 1) {
            PARK_DIST = 3;
            ROTATION = -90;
        }
        else if (tagOfInterest.id == 2) {
            PARK_DIST = 30;
            ROTATION = -90;
        }
        else return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(SPEED_LIMIT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SPEED_LIMIT)
                )
                .forward(FORWARD)
                .strafeLeft(STRAFE)
                .addTemporalMarker(-0.25, () -> {
                    glis.setToPosition(4);
                })
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    turela.setToTicks((int) TURELA_TICKS, PRELOAD.TURELA_POWER);
                    foarfeca.setToPosition(PRELOAD.FOARFECA_POS);
                })
                .waitSeconds(PRELOAD.WAIT_TIME)
                .addTemporalMarker(() -> {
                    colectare.desface();
                    foarfeca.setToPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turela.setToTicks(CYCLE.CONE_TICKS, 0.7);
                    foarfeca.setToPosition(0.5);

                    glis.setToTicks((int) CYCLE.GLIS_POS);
                })
                .waitSeconds(1.7)
                .addTemporalMarker(() -> foarfeca.setToPosition(CYCLE.FOARFECA_POS))
                .UNSTABLE_addTemporalMarkerOffset(1, colectare::strange)
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> glis.setToPosition(4))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    foarfeca.setToPosition(0.5);
                    turela.setToTicks((int) TURELA_TICKS, CYCLE.TURELA_POWER);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> foarfeca.setToPosition(PRELOAD.FOARFECA_POS))
                .UNSTABLE_addTemporalMarkerOffset(3 + CYCLE.WAIT_TIME, colectare::desface)
                .UNSTABLE_addTemporalMarkerOffset(3.5 + CYCLE.WAIT_TIME, () -> {
                    turela.setToTicks((int) (CYCLE.CONE_TICKS), 0.7);
                    foarfeca.setToPosition(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(3.75 + CYCLE.WAIT_TIME, () -> glis.setToTicks((int) (CYCLE.GLIS_POS - CYCLE.GLIS_OFFSET)))
                .waitSeconds(7)
                .addTemporalMarker(() -> foarfeca.setToPosition(CYCLE.FOARFECA_POS))
                .UNSTABLE_addTemporalMarkerOffset(1, colectare::strange)
                .UNSTABLE_addTemporalMarkerOffset(1.75, () -> glis.setToPosition(4))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    foarfeca.setToPosition(0.5);
                    turela.setToTicks((int) TURELA_TICKS, CYCLE.TURELA_POWER);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> foarfeca.setToPosition(PRELOAD.FOARFECA_POS))
                .UNSTABLE_addTemporalMarkerOffset(3 + CYCLE.WAIT_TIME, colectare::desface)
                .UNSTABLE_addTemporalMarkerOffset(3.5 + CYCLE.WAIT_TIME, () -> {
                    turela.setToPosition(Turela.Position.FRONT, 0.7);
                    foarfeca.setToPosition(0.5);
                })
                .waitSeconds(6)
                .turn(Math.toRadians(ROTATION))
                .forward(PARK_DIST)
                .addTemporalMarker(() -> glis.setToPosition(0))
                .resetConstraints()
                .build();

        colectare.toggleDeget();
        foarfeca.setToPosition(0.5);
        sleep(650);

        glis.setToPosition(1);
        drive.followTrajectorySequence(traj);
    }

    public static class Preload {
        public double TURELA_POWER, FOARFECA_POS, WAIT_TIME;

        public Preload(double turelaPower, double foarfecaPos, double waitTime) {
            this.TURELA_POWER = turelaPower;
            this.FOARFECA_POS = foarfecaPos;
            this.WAIT_TIME = waitTime;
        }
    }

    public static class Cycle {
        public double TURELA_POWER, FOARFECA_POS;
        public double GLIS_POS, GLIS_OFFSET, WAIT_TIME;
        public int CONE_TICKS;

        public Cycle(double turelaPower, double foarfecaPos, int coneTicks, double glisPos, double offset, double waitTime) {
            this.TURELA_POWER = turelaPower;
            this.FOARFECA_POS = foarfecaPos;
            this.GLIS_POS = glisPos;
            this.GLIS_OFFSET = offset;
            this.CONE_TICKS = coneTicks;
            this.WAIT_TIME = waitTime;
        }
    }

    String getPosition(int id)
    {
        if (id == 0)
            return "Left";
        else if (id == 1)
            return "Middle";
        else if (id == 2)
            return "Right";

        return "A strange bug has appeared!";
    }
}
