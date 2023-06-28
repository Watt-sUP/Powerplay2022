package org.firstinspires.ftc.teamcode.commands.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Hashtable;
import java.util.function.Supplier;

public class TurelaSubsystem extends SubsystemBase {
    private final DcMotor motor;
    private final Hashtable<Direction, Integer> posDict;
    private boolean useSafety;
    private Supplier<Boolean> safeSupplier;
    public Direction lastDirection = Direction.FORWARD;

    /**
     * Initializes the subsystem without safety checks.
     *
     * @param turelaMotor Motor object, used to control the turret
     */
    public TurelaSubsystem(@NonNull Motor turelaMotor) {
        this.motor = turelaMotor.motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.posDict = new Hashtable<>();
        useSafety = false;

        posDict.put(Direction.LEFT, 800);
        posDict.put(Direction.RIGHT, -800);
        posDict.put(Direction.FORWARD, 0);
    }

    /**
     * Initializes the subsystem with safety checks to avoid damage.
     *
     * @param turelaMotor  Motor object, used to control the turret
     * @param safeSupplier Supplier that returns true if the turret is safe to move
     */
    public TurelaSubsystem(@NonNull Motor turelaMotor, Supplier<Boolean> safeSupplier) {
        this(turelaMotor);
        this.safeSupplier = safeSupplier;
        useSafety = true;
    }

    /**
     * Moves the turret to a given direction.
     *
     * @param direction Direction to move the turret to (LEFT, RIGHT, FORWARD)
     * @param power     Power value to move the turret with
     */
    public void setToPosition(Direction direction, double power) {
        if (useSafety && !safeSupplier.get()) {
            return;
        }

        if (direction != Direction.FORWARD) {
            lastDirection = direction;
        }

        motor.setTargetPosition(posDict.get(direction));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     * Moves the turret to a given direction with a default power value of 1.
     *
     * @param direction Direction to move the turret to (LEFT, RIGHT, FORWARD)
     * @see #setToPosition(Direction direction, double power)
     */
    public void setToPosition(Direction direction) {
        setToPosition(direction, 1.0);
    }

    /**
     * Moves the turret to a given number of ticks.
     *
     * @param ticks Number of ticks to move the turret to
     * @param power Power value to move the turret with
     */
    public void setToTicks(int ticks, double power) {
        if (useSafety && !safeSupplier.get()) {
            return;
        }

        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     * Moves the turret to a given number of ticks with a default power value of 1.
     *
     * @param ticks Number of ticks to move the turret to
     * @see #setToTicks(int ticks, double power)
     */
    public void setToTicks(int ticks) {
        setToTicks(ticks, 1.0);
    }

    /**
     * Alters the target position of the turret by a given number of ticks. Also accepts negative values.
     *
     * @param ticks Number of ticks to add to the current target position
     * @param power Power value to move the turret with
     */
    public void modifyTicks(int ticks, double power) {
        setToTicks(motor.getCurrentPosition() + ticks, power);
    }

    /**
     * <p>Alters the target position of the turret by a given number of ticks. Also accepts negative values.</p>
     * <p>Power value is defaulted to 1.</p>
     *
     * @param ticks Number of ticks to add to the current target position
     * @see #modifyTicks(int ticks, double power)
     */
    public void modifyTicks(int ticks) {
        modifyTicks(ticks, 1.0);
    }

    /**
     * Gets the current position of the turret.
     *
     * @return Turret position measured in ticks
     */
    public int getTicks() {
        return motor.getCurrentPosition();
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     *
     * @return True if busy, false otherwise
     */
    public boolean isBusy() {
        return motor.isBusy();
    }
}
