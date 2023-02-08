package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Helper subsystem for operating drivetrain motors.
 */
public class DriveMotors {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    /**
     * <p>Creates a new instance of the class.</p>
     * <p>Initializes the drivetrain motors from the config.</p>
     * @param hardwareMap The hardware map needed to get the motors of the drivetrain
     */
    public DriveMotors(@NonNull HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get(Config.left_front);
        frontRight = hardwareMap.dcMotor.get(Config.right_front);
        backLeft = hardwareMap.dcMotor.get(Config.left_back);
        backRight = hardwareMap.dcMotor.get(Config.right_back);
    }

    /**
     * <p>Reverses 2 of the drivetrain's motors.</p>
     * <p>Passing in an invalid side will result in no operation occuring.</p>
     * @param side Takes 2 values: left or right (case independent), depending on the desired part to be reversed.
     */
    public void reverse_motors(@NonNull String side) {
        if (side.equalsIgnoreCase("left")) {
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (side.equalsIgnoreCase("right")) {
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    /**
     * <p>Updates the motor speeds based on the gamepad input.</p>
     * <p>Control axes are the left stick for forward/backward and strafing, and the right stick for rotation.</p>
     * <p>Passing in a null limit will result in no limit being applied.</p>
     * @param gamepad The gamepad to get the input from
     * @param limit Optional parameter to limit the motor speeds
     */
    public void update_motor_speed(@NonNull Gamepad gamepad, @Nullable Double limit) {
        double accel = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x * 1.1;
        double rotation = gamepad.right_stick_x;
        double FL_power, BL_power, FR_power, BR_power;
        double denominator = Math.max(Math.abs(accel) + Math.abs(strafe) + Math.abs(rotation), 1);

        FL_power = (accel + strafe + rotation) / denominator;
        BL_power = (accel - strafe + rotation) / denominator;
        FR_power = (accel - strafe - rotation) / denominator;
        BR_power = (accel + strafe - rotation) / denominator;

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