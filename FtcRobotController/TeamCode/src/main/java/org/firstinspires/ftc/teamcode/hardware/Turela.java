package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Hashtable;
import java.util.function.Supplier;

public class Turela {

    private final Hashtable<Direction, Integer> pos_dict;
    private boolean useSafety;
    private Supplier<Boolean> safeSupplier;
    public DcMotorEx motortur;

    /**
     * Initializes the motor.
     *
     * @param hardwareMap Hardware map to load the motor
     */
    public Turela(@NonNull HardwareMap hardwareMap) {
        motortur = hardwareMap.get(DcMotorEx.class, Config.turela);
        motortur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        useSafety = false;

        pos_dict = new Hashtable<>();
        pos_dict.put(Direction.FORWARD, 0);
        pos_dict.put(Direction.LEFT, 2100);
        pos_dict.put(Direction.RIGHT, -2000);
        pos_dict.put(Direction.BACKWARDS, -4100);
    }

    /**
     * Moves the turret to the given position.
     *
     * @param position The desired position
     * @param power The desired power value
     */
    public void setToPosition(Direction position, double power) {
        if (useSafety && !safeSupplier.get()) {
            return;
        }

        motortur.setTargetPosition(pos_dict.get(position));
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortur.setPower(power);
    }

    /**
     * Moves the turret to the given position.
     * @param position The desired position
     */
    public void setToPosition(Direction position) {
        setToPosition(position, 1);
    }

    /**
     * Moves the turret based on the supplied ticks.
     * @param ticks The ticks to go to
     * @param power The desired power value
     */
    public void setToTicks(int ticks, double power) {
        if (useSafety && !safeSupplier.get()) {
            return;
        }

        motortur.setTargetPosition(ticks);
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortur.setPower(power);
    }

    /**
     * Moves the turret based on the supplied ticks.
     * @param ticks The ticks to go to
     */
    public void setToTicks(int ticks) {
        setToTicks(ticks, 1);
    }

    /**
     * Helper method to log the tick count.
     *
     * @return Motor tick count
     */
    public int getTicks() {
        return motortur.getCurrentPosition();
    }

    /**
     * Enables a safety mechanism that avoids moving the turret whenever the slides are down.
     * @param safeSupplier A supplier that returns true if the slides aren't down
     */
    public void enableSafety(Supplier<Boolean> safeSupplier) {
        useSafety = true;
        this.safeSupplier = safeSupplier;
    }

    /**
     * Disables the safety mechanism.
     */
    public void disableSafety() {
        useSafety = false;
        safeSupplier = null;
    }
}
