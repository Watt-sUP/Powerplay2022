package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.DriveMotors;

import java.text.DecimalFormat;

@TeleOp(name = "IMU Angle Test", group = "Testing")
public class IMUTest extends LinearOpMode {

    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    Servo odometry_servo = hardwareMap.servo.get(Config.odometry_servo);
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    DecimalFormat df = new DecimalFormat("0.00");

    @Override
    public void runOpMode() throws InterruptedException {

        DriveMotors driveMotors = new DriveMotors(hardwareMap);
        driveMotors.reverse_motors("Right");
        odometry_servo.setPosition(0.6);

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        if(isStopRequested())
            return;

        while (opModeIsActive()) {
            double botHeading = -imu.getAngularOrientation().firstAngle;
            driveMotors.update_motor_speed(gamepad1, null);

            telemetry.addData("Robot Heading (Radians):", df.format(botHeading));
            telemetry.addData("Robot Heading (Degrees):", df.format(Math.toDegrees(botHeading)));
            telemetry.update();
        }
    }
}
