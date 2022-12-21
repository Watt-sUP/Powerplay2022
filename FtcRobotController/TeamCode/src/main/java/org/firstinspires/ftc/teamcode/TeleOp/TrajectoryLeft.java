package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Glisiere;
import org.firstinspires.ftc.teamcode.hardware.Turela;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.text.DecimalFormat;
import java.util.List;

@Config
@Autonomous(name = "Traiectorie Stanga", group = "auto")
public class TrajectoryLeft extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/modelSCT_2.tflite";
    public static double ZOOM = 1.7, CONFIDENCE = 0.33;
    private String detected_obj = null;
    private double confidence, last_confidence = 0.0;
    private static final DecimalFormat df = new DecimalFormat("0.00");
    private static final String[] LABELS = {
            "Circle",
            "Square",
            "Triangle",
    };

    public static int FORWARD = 50;
    public static int STRAFE = 24;
    public static int SPEED_LIMIT = 45, TURN_LIMIT = 4;
    private static final String VUFORIA_KEY = org.firstinspires.ftc.teamcode.hardware.Config.VuforiaKey;
    public static double DEGET_TIME = 6.5, FORWARD_OFFSET = 10, BACK_OFFSET = 12, PARK_DIST = 25, GLIS_OFFSET = 1.5, TURN = -90;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(ZOOM, 16.0 / 9.0);
        }

        while (!isStarted() && tfod != null) {
            if (isStopRequested())
                return;
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            confidence = 0.0;
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions)
                    if (recognition.getConfidence() > confidence) {
                        confidence = recognition.getConfidence();
                        last_confidence = confidence;
                        detected_obj = recognition.getLabel();
                    }
            }
            if (last_confidence <= 1)
                last_confidence = last_confidence * 100;
            telemetry.addData("Last detected object", (detected_obj != null) ? detected_obj : "N/A");
            telemetry.addLine("Detection confidence: " + df.format(last_confidence) + "%");
            telemetry.update();
        }

        if (tfod != null)
            tfod.deactivate();

        Colectare deget = new Colectare(hardwareMap);
        Glisiere glis = new Glisiere(hardwareMap);
        Turela tur = new Turela(hardwareMap);

        if (detected_obj == "Square")
            PARK_DIST = 47;
        else if (detected_obj == "Triangle")
            PARK_DIST = 5;
        else if (detected_obj == "Circle")
            PARK_DIST = 25;

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
                .strafeRight(BACK_OFFSET)
                .waitSeconds(1.5)
                .strafeLeft(FORWARD_OFFSET)
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

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) CONFIDENCE;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}