package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

        DriveMotors driveMotors = new DriveMotors(hardwareMap);
        Servo odometry_servo = hardwareMap.servo.get(Config.odometry_servo);
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        driveMotors.reverse_motors("Right");
        odometry_servo.setPosition(Config.odo_pos);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

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

            telemetry.addData("Bot heading", df.format(imu.getAngularOrientation().firstAngle));
            telemetry.addData("IMU second angle", df.format(imu.getAngularOrientation().secondAngle));
            telemetry.addData("IMU third angle", df.format(imu.getAngularOrientation().thirdAngle));
            telemetry.update();
        }
    }
}
