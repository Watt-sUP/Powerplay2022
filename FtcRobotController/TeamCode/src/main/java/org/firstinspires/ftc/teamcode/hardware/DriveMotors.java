package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Objects;

public class DriveMotors {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public DriveMotors(@NonNull HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get(Config.left_front);
        frontRight = hardwareMap.dcMotor.get(Config.right_front);
        backLeft = hardwareMap.dcMotor.get(Config.left_back);
        backRight = hardwareMap.dcMotor.get(Config.right_back);
    }

    public void reverse_motors(@NonNull String side) {
        if (Objects.equals(side.toLowerCase(), "left")) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (Objects.equals(side.toLowerCase(), "right")) {
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void update_motor_speed(@NonNull Gamepad gamepad, @Nullable Double limit, @Nullable Double botHeading) {
        double accel = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x * 1.1;
        double rotation = gamepad.right_stick_x;
        double FL_power, BL_power, FR_power, BR_power;
        double denominator = Math.max(Math.abs(accel) + Math.abs(strafe) + Math.abs(rotation), 1);

        if (botHeading != null) {
            double rotX = strafe * Math.cos(botHeading) - accel * Math.sin(botHeading);
            double rotY = strafe * Math.sin(botHeading) + accel * Math.cos(botHeading);

            FL_power = (rotY + rotX + rotation) / denominator;
            BL_power = (rotY - rotX + rotation) / denominator;
            FR_power = (rotY - rotX - rotation) / denominator;
            BR_power = (rotY + rotX - rotation) / denominator;
        }

        else {
            FL_power = (accel + strafe + rotation) / denominator;
            BL_power = (accel - strafe + rotation) / denominator;
            FR_power = (accel - strafe - rotation) / denominator;
            BR_power = (accel + strafe - rotation) / denominator;
        }

        if (limit != null) {
            FL_power = Range.clip(FL_power, -limit, limit);
            FR_power = Range.clip(FR_power, -limit, limit);
            BL_power = Range.clip(BL_power, -limit, limit);
            BR_power = Range.clip(BR_power, -limit, limit);
        }

        frontLeft.setPower(FL_power);
        frontRight.setPower(FR_power);
        backLeft.setPower(BL_power);
        backRight.setPower(BR_power);
    }
}