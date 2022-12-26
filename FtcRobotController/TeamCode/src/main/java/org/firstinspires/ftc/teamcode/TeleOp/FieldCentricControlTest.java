package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.DriveMotors;

import java.text.DecimalFormat;

@TeleOp
public class FieldCentricControlTest extends LinearOpMode {
    DecimalFormat df = new DecimalFormat("0.00");

    @Override
    public void runOpMode() throws InterruptedException {
        DriveMotors driveMotors = new DriveMotors(hardwareMap);
        Servo odometry_servo = hardwareMap.servo.get(Config.odometry_servo);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        driveMotors.reverse_motors("right");
        odometry_servo.setPosition(Config.odo_pos);

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            double botHeading = imu.getAngularOrientation().firstAngle;

            driveMotors.update_motor_speed(gamepad1, null, botHeading * (-1));
            telemetry.addData("Bot Heading (Radians)", df.format(botHeading));
            telemetry.addData("Bot Heading (Degrees)", df.format(Math.toDegrees(botHeading)));
            telemetry.update();
        }
    }
}
