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

    /**
     * Initializes the subsystem without safety checks.
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
        posDict.put(Direction.BACKWARDS, 1610);
    }

    /**
     * Initializes the subsystem with safety checks to avoid damage.
     * @param turelaMotor Motor object, used to control the turret
     * @param safeSupplier Supplier that returns true if the turret is safe to move
     */
    public TurelaSubsystem(@NonNull Motor turelaMotor, Supplier<Boolean> safeSupplier) {
        this(turelaMotor);
        this.safeSupplier = safeSupplier;
        useSafety = true;
    }

    /**
     * Moves the turret to a given direction.
     * @param direction Direction to move the turret to (LEFT, RIGHT, FORWARD, BACKWARDS)
     * @param power Power value to move the turret with
     */
    public void setToPosition(Direction direction, double power) {
        if (useSafety && !safeSupplier.get()) {
            return;
        }

        motor.setTargetPosition(posDict.get(direction));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     * Moves the turret to a given number of ticks.
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
     * Alters the current position of the turret by a given number of ticks.
     * @param ticks Number of ticks to change the turret position by
     * @param power Power value to move the turret with
     */
    public void modifyByTicks(int ticks, double power) {
        if (useSafety && !safeSupplier.get()) {
            return;
        }

        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void setToPosition(Direction direction) {
        setToPosition(direction, 1.0);
    }
    public void setToTicks(int ticks) {
        setToTicks(ticks, 1.0);
    }
    public void modifyByTicks(int ticks) {
        modifyByTicks(ticks, 0.5);
    }
    public int getTicks() {
        return motor.getCurrentPosition();
    }
}
