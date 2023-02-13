package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Command-based subsystem for the sliders mechanism.
 */
public class GlisiereSubsystem extends SubsystemBase {
    public int position;
    private final int[] positions = {0, 300, 725, 1300, 1835};
    private final DcMotor motor, motor2;

    /**
     * Creates a new instance of the subsystem.
     * @param motor DcMotor object controlling the first motor
     * @param motor2 DcMotor object controlling the second motor
     */
    public GlisiereSubsystem(DcMotor motor, DcMotor motor2) {
        this.motor = motor;
        this.motor2 = motor2;

        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Moves the sliders to the given position.
     * @param position The desired position (ranges 0-4)
     */
    public void setToPosition(int position) {

        this.position = position;
        this.position = MathUtils.clamp(this.position, 0, 4);

        motor.setTargetPosition(positions[this.position]);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.98);

        motor2.setTargetPosition(positions[this.position]);
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
     * @param ticks The ticks count to change the current position by
     */
    public void modifyTicks(int ticks) {
        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.98 * 0.2);

        motor2.setTargetPosition(motor2.getCurrentPosition() + ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(0.2);
    }

    /**
     * Helper method for turret safety.
     * @return The current position of the sliders
     */
    public int getPosition() {
        return position;
    }

    public int getTicks() {
        return motor.getCurrentPosition();
    }
}
