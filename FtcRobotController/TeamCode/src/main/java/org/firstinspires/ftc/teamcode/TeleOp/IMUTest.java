package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Config;

import java.text.DecimalFormat;

@TeleOp(name = "IMU Angle Test", group = "Testing")
public class IMUTest extends LinearOpMode {
    DecimalFormat df = new DecimalFormat("0.00");

    @Override
    public void runOpMode() throws InterruptedException {


        Motor frontLeft = new Motor(hardwareMap, Config.left_front);
        Motor frontRight = new Motor(hardwareMap, Config.right_front);
        Motor backLeft = new Motor(hardwareMap, Config.left_back);
        Motor backRight = new Motor(hardwareMap, Config.right_back);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        Servo odometry_servo = hardwareMap.servo.get(Config.odometry_servo);
        odometry_servo.setPosition(Config.odo_pos);

        RevIMU imu = new RevIMU(hardwareMap, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.init(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Odometry servo raised to " + Config.odo_pos);
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        if(isStopRequested())
            return;

        while (opModeIsActive()) {

            drive.driveRobotCentric(
                    gamepad.getLeftX(),
                    gamepad.getLeftY(),
                    gamepad.getRightX(),
                    false
            );
            gamepad.readButtons();

            if (gamepad.wasJustPressed(GamepadKeys.Button.A))
                imu.reset();

            if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
                drive.setMaxSpeed(0.2);
            else if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
                drive.setMaxSpeed(0.4);
            else drive.setMaxSpeed(1.0);

            telemetry.addData("Bot Heading (Relative)", df.format((imu.getHeading() < 0) ? (imu.getHeading() + 360) : imu.getHeading()));
            telemetry.addData("Bot Heading (Absolute)", df.format((imu.getAbsoluteHeading() < 0) ? (imu.getAbsoluteHeading() + 360) : imu.getAbsoluteHeading()));
            telemetry.addData("Acceleration Value", df.format(gamepad.getLeftY()));
            telemetry.addData("Strafe Value", df.format(gamepad.getLeftX()));
            telemetry.addData("Rotation Value", df.format(gamepad.getRightX()));
            telemetry.update();
        }
    }
}
