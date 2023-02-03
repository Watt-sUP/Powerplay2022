package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Hashtable;

public class Turela {

    private final Hashtable<Direction, Integer> pos_dict;
    public int glis_position = 1;
    public MotorEx motortur;
    private PIDController pidController;

    /**
     * Initializes the motor.
     *
     * @param hardwareMap Hardware map to load the motor
     */
    public Turela(HardwareMap hardwareMap) {
        motortur = new MotorEx(hardwareMap, Config.turela);
        motortur.setInverted(true);
        motortur.setRunMode(Motor.RunMode.RawPower);

        pidController = new PIDController(4, 3, 0);
        pidController.setSetPoint(0);
        pidController.setTolerance(0);

        pos_dict = new Hashtable<>();
        pos_dict.put(Direction.FORWARD, 0);
        pos_dict.put(Direction.LEFT, 2100);
        pos_dict.put(Direction.RIGHT, -2000);
        pos_dict.put(Direction.BACKWARDS, -4100);
    }

    /**
     * Updates the kP value based on the supplied position.
     *
     * @param position The desired position
     */
    public void setToPosition(Direction position) {
        pidController.setSetPoint(pos_dict.get(position));
    }

    /**
     * <p>Updates the kP value based on ticks.</p>
     *
     * @param ticks The ticks to go to
     */
    public void setToTicks(int ticks) {
        pidController.setSetPoint(ticks);
    }

    /**
     * Updates the power once to avoid blocking the thread.
     */
    public void updatePower() {
        double output = pidController.calculate(-motortur.getCurrentPosition());
        motortur.setVelocity(output);
    }

    /**
     * Helper method to log the tick count
     *
     * @return Motor tick count
     */
    public int getTicks() {
        return motortur.getCurrentPosition();
    }
}
