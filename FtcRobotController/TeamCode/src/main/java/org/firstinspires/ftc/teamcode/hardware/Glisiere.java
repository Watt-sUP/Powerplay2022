package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper subsystem for operating the sliders mechanism.
 */
public class Glisiere {

    public DcMotor motor;
    public DcMotor motor2;
    private final int[] positions = {0, 450, 975, 1650, 2360};

    /**
     * Creates a new instance of the sliders mechanism.
     * @param hardwareMap The hardware map to load the motors
     */
    public Glisiere(@NonNull HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get(Config.glisiera);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2 = hardwareMap.dcMotor.get(Config.glisiera1);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Moves the sliders to the given position.
     * @param position The desired position (ranges 0-4)
     */
    public void setToPosition(int position) {
        motor.setTargetPosition(positions[position]);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.98);

        motor2.setTargetPosition(positions[position]);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
    }

    /**
     * Moves the sliders in an absolute manner based on a custom ticks value.
     * @param ticks The ticks amount to move to
     */
    public void setToTicks(int ticks) {
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.98);

        motor2.setTargetPosition(ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
    }

    /**
     * Moves the sliders in a relative manner based on a custom ticks value.
     * @param ticks The ticks amount to change the position by
     */
    public void modifyPosition(int ticks) {
        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2 * 0.98);

        motor2.setTargetPosition(motor2.getCurrentPosition() + ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(0.2);
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public int getOffset() {
        return Math.abs(motor.getCurrentPosition() - motor2.getCurrentPosition());
    }
}
