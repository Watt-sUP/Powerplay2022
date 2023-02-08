package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Helper subsystem for operating the scissor mechanism.
 */
public class Foarfeca {
    private final double retractedPos;
    private StateFoarfeca stateFoarfeca;
    public ServoEx foarfeca;

    private enum StateFoarfeca {
        Retracted,
        Extended,
        Custom
    }

    /**
     * Creates a new instance of the scissor mechanism.
     * @param hardwareMap The hardware map to load the motor
     * @param init_pos The initial position of the servo (0.2 by default)
     */
    public Foarfeca(HardwareMap hardwareMap, double init_pos) {
        foarfeca = new SimpleServo(hardwareMap, Config.foarfeca, 0, 360);
        foarfeca.setInverted(true);
        foarfeca.setPosition(init_pos);
        retractedPos = init_pos;
        stateFoarfeca = StateFoarfeca.Retracted;
    }

    /**
     * Creates a new instance of the scissor mechanism.
     * @param hardwareMap The hardware map to load the motor
     */
    public Foarfeca(HardwareMap hardwareMap) {
        this(hardwareMap, 0.2);
    }

    /**
     * Sets the scissor mechanism to a custom position.
     * @param position The position to set the servo to
     */
    public void setToPosition(double position) {
        foarfeca.setPosition(position);
        stateFoarfeca = StateFoarfeca.Custom;
    }

    /**
     * Extends the scissor mechanism.
     */
    public void extend() {
        double extendedPos = 0.75;
        foarfeca.setPosition(extendedPos);
        stateFoarfeca = StateFoarfeca.Extended;
    }

    /**
     * Retracts the scissor mechanism.
     */
    public void retract() {
        foarfeca.setPosition(retractedPos);
        stateFoarfeca = StateFoarfeca.Retracted;
    }

    /**
     * Toggles the scissor mechanism between extended and retracted.
     */
    public void toggleFoarfeca() {
        if (stateFoarfeca == StateFoarfeca.Extended || stateFoarfeca == StateFoarfeca.Custom) retract();
        else extend();
    }
}
