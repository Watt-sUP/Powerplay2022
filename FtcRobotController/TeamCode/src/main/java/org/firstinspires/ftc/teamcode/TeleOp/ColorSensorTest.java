package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@Disabled
@TeleOp
public class ColorSensorTest extends LinearOpMode {
    SensorColor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: Insert Sensor Name
        sensor = new SensorColor(hardwareMap, "name");
        float[] hsv = {-1, -1, -1};

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        while (opModeIsActive()) {
            sensor.RGBtoHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);
            telemetry.addData("RGB Value:", Arrays.asList(sensor.red(), sensor.green(), sensor.blue()));
            telemetry.addData("HSV Value:", hsv);
            telemetry.update();
        }
    }

}
