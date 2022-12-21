package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.Config;

import java.text.DecimalFormat;
import java.util.List;

@Autonomous(name = "Autonom Ploiesti", group = "auto")
public class AutonomPloiesti extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/modelSCT.tflite";
    private DecimalFormat df = new DecimalFormat("0.00");
    private String detected_obj = null;
    private double confidence, last_confidence;
    private static final String[] LABELS = {
            "Circle",
            "Square",
            "Triangle",
    };

    private static final String VUFORIA_KEY = Config.VuforiaKey;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Config.left_front);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Config.left_back);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Config.right_front);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Config.right_back);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        while (!isStarted() && tfod != null) {
            if(isStopRequested())
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
            telemetry.addData("Last detected", (detected_obj != null) ? detected_obj : "N/A");
            telemetry.addData("Confidence", df.format(last_confidence));
            telemetry.update();
        }

        tfod.deactivate();
        if (detected_obj  == "Triangle") {
            frontRightMotor.setPower(-0.4);
            frontLeftMotor.setPower(0.4);
            backLeftMotor.setPower(-0.4);
            backRightMotor.setPower(0.4);
            sleep(1200);
        }
        else if (detected_obj == "Square") {
            frontRightMotor.setPower(0.4);
            frontLeftMotor.setPower(-0.4);
            backLeftMotor.setPower(0.4);
            backRightMotor.setPower(-0.4);
            sleep(1330);
        }

        if (detected_obj != null) {
            frontRightMotor.setPower(-0.4);
            frontLeftMotor.setPower(-0.4);
            backLeftMotor.setPower(-0.4);
            backRightMotor.setPower(-0.4);
            sleep(500);
        }
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
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}