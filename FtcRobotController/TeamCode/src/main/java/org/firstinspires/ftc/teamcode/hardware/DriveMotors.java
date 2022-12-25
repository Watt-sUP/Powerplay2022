package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class DriveMotors {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public DriveMotors(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get(Config.left_front);
        frontRight = hardwareMap.dcMotor.get(Config.right_front);
        backLeft = hardwareMap.dcMotor.get(Config.left_back);
        backRight = hardwareMap.dcMotor.get(Config.right_back);
    }

    public void reverse_motors(String side) {
        if (side.toLowerCase() == "left") {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (side.toLowerCase() == "right") {
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void update_motor_speed(@NonNull Gamepad gamepad, @Nullable Double limit) {
        double accel = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x * 1.1;
        double rotation = gamepad.left_stick_x;

        double denominator = Math.max(Math.abs(accel) + Math.abs(strafe) + Math.abs(rotation), 1);
        double FL_power = (accel + strafe + rotation) / denominator;
        double BL_power = (accel - strafe + rotation) / denominator;
        double FR_power = (accel - strafe - rotation) / denominator;
        double BR_power = (accel + strafe - rotation) / denominator;

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