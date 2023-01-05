package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.DriveMotors;

import java.text.DecimalFormat;

@TeleOp(name = "IMU Angle Test", group = "Testing")
public class IMUTest extends LinearOpMode {
    DecimalFormat df = new DecimalFormat("0.00");

    @Override
    public void runOpMode() throws InterruptedException {

        PhotonCore.enable();
        GamepadEx gamepad = new GamepadEx(gamepad1);

        DriveMotors driveMotors = new DriveMotors(hardwareMap);
        Servo odometry_servo = hardwareMap.servo.get(Config.odometry_servo);
        driveMotors.reverse_motors("Right");
        odometry_servo.setPosition(Config.odo_pos);

        RevIMU imu = new RevIMU(hardwareMap, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.init(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Odometry servo raised to" + Config.odo_pos);
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if(isStopRequested())
            return;

        while (opModeIsActive()) {
            driveMotors.update_motor_speed(gamepad1, null, null);

            gamepad.readButtons();
            if (gamepad.wasJustPressed(GamepadKeys.Button.A))
                imu.reset();

            telemetry.addData("Bot Heading (Relative)", df.format(imu.getHeading()));
            telemetry.addData("Bot Heading (Absolute)", df.format(imu.getAbsoluteHeading()));
            telemetry.addData("Acceleration Value", df.format(gamepad.getLeftY()));
            telemetry.addData("Strafe Value", df.format(gamepad.getLeftX()));
            telemetry.addData("Rotation Value", df.format(gamepad.getRightX()));
            telemetry.update();
        }
        PhotonCore.disable();
    }
}
