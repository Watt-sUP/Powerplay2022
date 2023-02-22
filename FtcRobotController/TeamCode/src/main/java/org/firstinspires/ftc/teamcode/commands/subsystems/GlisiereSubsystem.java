package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
    private final int[] positions = {0, 300, 775, 1300, 1835};
    private final DcMotor motor, motor2;
    private final ServoEx ghidaj;
    private final Command ghidajDelay;

    private enum StateGhidaj {
        Active,
        Inactive
    }

    private StateGhidaj stateGhidaj;

    /**
     * Creates a new instance of the subsystem.
     *
     * @param motor  DcMotor object controlling the first motor
     * @param motor2 DcMotor object controlling the second motor
     */
    public GlisiereSubsystem(DcMotor motor, DcMotor motor2, ServoEx ghidaj) {
        this.motor = motor;
        this.ghidaj = ghidaj;
        this.motor2 = motor2;

        this.ghidajDelay = new WaitUntilCommand(() -> motor.getCurrentPosition() >= positions[2])
                .andThen(new InstantCommand(() -> {
                            ghidaj.turnToAngle(180);
                            stateGhidaj = StateGhidaj.Active;
                        })
                );

        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.ghidaj.setInverted(false);
        closeGhidaj();
    }

    /**
     * Moves the sliders to the given position.
     *
     * @param position The desired position (ranges 0-4)
     */
    public void setToPosition(int position) {
        this.position = MathUtils.clamp(position, 0, 4);

        if (this.position >= 3) openGhidaj();
        else closeGhidaj();

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
        if (ticks >= positions[3]) openGhidaj();
        else closeGhidaj();

        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        motor2.setTargetPosition(ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
    }

    /**
     * Moves the sliders in a relative manner based on a custom ticks value.
     *
     * @param ticks The ticks count to change the current position by
     */
    public void modifyTicks(int ticks) {
        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2);

        motor2.setTargetPosition(motor2.getCurrentPosition() + ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(0.2);
    }

    public void openGhidaj() {
        ghidajDelay.schedule(true);
    }

    public void closeGhidaj() {
        if (ghidajDelay.isScheduled())
            ghidajDelay.cancel();

        if (stateGhidaj == StateGhidaj.Inactive)
            return;

        this.ghidaj.turnToAngle(0);
        stateGhidaj = StateGhidaj.Inactive;
    }

    public void toggleGhidaj() {
        if (stateGhidaj == StateGhidaj.Inactive) openGhidaj();
        else closeGhidaj();
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
