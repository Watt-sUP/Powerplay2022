package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Command-based subsystem for the sliders mechanism.
 */
public class GlisiereSubsystem extends SubsystemBase {
    public int position;
    /*
     * The positions of the sliders, in encoder ticks.
     * Positions Legend:
     * 0: Starting position
     * 1: Stack position
     * 2: Low junction
     * 3: Middle junction
     * 4: High junction
     */
    private final int[] positions = {0, 300, 575, 1150, 1675};
    private boolean use_automations = true;
    private final DcMotor motor, motor2;
    private final ServoEx unghi;
    private StateUnghi stateUnghi = StateUnghi.Lowered;
    private enum StateUnghi {
        Raised,
        Lowered
    }

    /**
     * Creates a new instance of the subsystem.
     *
     * @param motor  DcMotor object controlling the first motor
     * @param motor2 DcMotor object controlling the second motor
     */
    public GlisiereSubsystem(DcMotor motor, DcMotor motor2, ServoEx unghi) {
        this.motor = motor;
        this.motor2 = motor2;
        this.unghi = unghi;

        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.unghi.setInverted(true);
        this.unghi.turnToAngle(160);
    }

    public GlisiereSubsystem(DcMotor motor, DcMotor motor2, ServoEx unghi, boolean use_automations) {
        this(motor, motor2, unghi);
        this.use_automations = use_automations;
    }

    /**
     * Moves the sliders to the given position.
     *
     * @param position The desired position (ranges 0-4)
     */
    public void setToPosition(int position) {
        this.position = MathUtils.clamp(position, 0, 4);

        if (use_automations) {
            if (this.position >= 2) raiseUnghi();
            else lowerUnghi();
        }

        motor.setTargetPosition(positions[this.position]);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        motor2.setTargetPosition(positions[this.position]);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
    }

    /**
     * Moves the sliders in an absolute manner based on a custom ticks value.
     *
     * @param ticks The ticks amount to move to
     */
    public void setToTicks(int ticks) {

        if (use_automations) {
            if (ticks >= positions[2]) raiseUnghi();
            else lowerUnghi();
        }

        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        motor2.setTargetPosition(ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
    }

    public void turnUnghiToAngle(double degrees) {
        unghi.turnToAngle(degrees);
    }

    /**
     * Moves the sliders in a relative manner based on a custom ticks value.
     *
     * @param ticks The ticks count to change the current position by
     */
    public void modifyTicks(int ticks) {

        if (use_automations) {
            if (motor.getCurrentPosition() + ticks >= positions[2]) raiseUnghi();
            else lowerUnghi();
        }

        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2);

        motor2.setTargetPosition(motor2.getCurrentPosition() + ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(0.2);
    }

    public void raiseUnghi() {
        unghi.turnToAngle(110);
        stateUnghi = StateUnghi.Raised;
    }

    public void lowerUnghi() {
        unghi.turnToAngle(160);
        stateUnghi = StateUnghi.Lowered;
    }

    /**
     * Gets the current position of the sliders.
     *
     * @return Sliders position, measured in encoder ticks
     */
    public int getTicks() {
        return motor.getCurrentPosition();
    }
}
