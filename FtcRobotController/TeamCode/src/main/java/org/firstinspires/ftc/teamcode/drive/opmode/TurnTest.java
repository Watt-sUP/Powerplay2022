package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static int ANGLE = 180; // deg
    public static boolean LOAD_IMU = false;
    private RevIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Foarfeca foarfeca = new Foarfeca(hardwareMap);
        Colectare deget = new Colectare(hardwareMap);
        if (LOAD_IMU) {
            imu = new RevIMU(hardwareMap, "imu");
            imu.init();
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));

        if (LOAD_IMU) {
            telemetry.addData("IMU Angle:", imu.getHeading());
            telemetry.addData("Angle difference:", Math.abs(ANGLE - imu.getHeading()));
            telemetry.update();
        }
    }
}
