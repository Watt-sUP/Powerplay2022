package org.firstinspires.ftc.teamcode.autonom.parcari;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonom.apriltag.AprilTagDetector;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Map;

@Disabled
@Autonomous(name = "Autonom Ploiesti (Dreapta)")
public class AutonomDreapta extends LinearOpMode {

    public static double POWER = 0.5;
    public static int SLEEP = 1800, SLEEP_2 = 1000;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    private int last_tag = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        Colectare colectare = new Colectare(hardwareMap);
        Foarfeca foarfeca = new Foarfeca(hardwareMap);

        frontLeft = hardwareMap.dcMotor.get(Config.left_front);
        frontRight = hardwareMap.dcMotor.get(Config.right_front);
        backLeft = hardwareMap.dcMotor.get(Config.left_back);
        backRight = hardwareMap.dcMotor.get(Config.right_back);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AprilTagDetector detector = new AprilTagDetector(hardwareMap);

        detector.WIDTH = 1280;
        detector.HEIGHT = 720;
        detector.ORIENTATION = OpenCvCameraRotation.SIDEWAYS_LEFT;
        detector.GPU_ENABLED = true;

        detector.init();
        detector.setTargets(0, 1, 2);

        while (!opModeIsActive() && !isStopRequested()) {
            Map<String, Integer> detection = detector.getDetection();
            telemetry.addData("AprilTag Found", detection == null ? "No" : "Yes");
            telemetry.addData("Last Known Tag", last_tag);

            if (detection != null) {
                last_tag = detection.get("id");
                telemetry.addData("Detection X", detection.get("x"));
                telemetry.addData("Detection Y", detection.get("y"));
            }

            telemetry.update();
        }
        detector.closeAsync();

        int strafe_multiplier;
        switch (last_tag) {
            case 2:
                strafe_multiplier = -1;
                SLEEP = 750;
                SLEEP_2 = 1300;
                break;
            case 1:
                strafe_multiplier = 0;
                SLEEP = 700;
                break;
            case 0:
                strafe_multiplier = 1;
                SLEEP = 750;
                SLEEP_2 = 1000;
                break;
            default:
                return;
        }


        frontLeft.setPower(-POWER * strafe_multiplier);
        frontRight.setPower(POWER * strafe_multiplier);
        backLeft.setPower(POWER * strafe_multiplier);
        backRight.setPower(-POWER * strafe_multiplier);

        sleep(SLEEP_2);

        frontLeft.setPower(POWER);
        frontRight.setPower(POWER);
        backLeft.setPower(POWER);
        backRight.setPower(POWER);

        sleep(SLEEP);
    }

}
