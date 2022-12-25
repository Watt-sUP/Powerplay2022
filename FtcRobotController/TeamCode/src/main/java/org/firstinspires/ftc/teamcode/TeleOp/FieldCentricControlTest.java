package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.DriveMotors;

@TeleOp
public class FieldCentricControlTest extends LinearOpMode {

    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Servo odometry_servo = hardwareMap.servo.get(Config.odometry_servo);

    @Override
    public void runOpMode() throws InterruptedException {
        DriveMotors driveMotors = new DriveMotors(hardwareMap);
        driveMotors.reverse_motors("right");
        odometry_servo.setPosition(0.6);

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        // TODO: Determine which angle represents the heading
        while (opModeIsActive())
            driveMotors.update_motor_speed(gamepad1, null);
    }
}
