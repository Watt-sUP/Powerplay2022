package org.firstinspires.ftc.teamcode.commands.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Command-based subsystem for the claw and scissors.
 */
public class ColectareSubsystem extends SubsystemBase {
    private final ServoEx claw, scissors;
    private double scissorsRetractedPos = 0.18;
    private StateClaw stateClaw;
    private StateScissors stateScissors;

    private enum StateClaw {
        Opened,
        Closed
    }
    private enum StateScissors {
        Extended,
        Retracted,
        Custom
    }

    /**
     * Creates a new instance of the subsystem.
     * @param claw ServoEx object controlling the claw
     * @param scissors ServoEx object controlling the scissors
     */
    public ColectareSubsystem(@NonNull ServoEx claw, @NonNull ServoEx scissors) {
        this.claw = claw;
        this.scissors = scissors;

        this.claw.setInverted(true);
        this.scissors.setInverted(true);

        this.claw.setPosition(0.4);
        this.scissors.setPosition(scissorsRetractedPos);

        stateClaw = StateClaw.Opened;
        stateScissors = StateScissors.Retracted;
    }

    public ColectareSubsystem(@NonNull ServoEx claw, @NonNull ServoEx scissors, double scissorsInitPos) {
        this(claw, scissors);
        scissorsRetractedPos = scissorsInitPos;
        this.scissors.setPosition(scissorsRetractedPos);
    }
    public void closeClaw() {
        claw.setPosition(1);
        stateClaw = StateClaw.Closed;
    }

    public void openClaw() {
        claw.setPosition(0.4);
        stateClaw = StateClaw.Opened;
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    /**
     * Toggles the claw between the open and closed positions.
     */
    public void toggleClaw() {
        if (stateClaw == StateClaw.Opened) closeClaw();
        else openClaw();
    }

    public void extendScissors() {
        scissors.setPosition(0.75);
        stateScissors = StateScissors.Extended;
    }

    public void retractScissors() {
        scissors.setPosition(scissorsRetractedPos);
        stateScissors = StateScissors.Retracted;
    }

    /**
     * Sets the scissors to a custom position.
     * @param position The position to set the scissors to
     */
    public void setScissorsPosition(double position) {
        scissors.setPosition(position);
        stateScissors = StateScissors.Custom;
    }

    /**
     * Toggles the scissors between the extended and retracted positions.
     */
    public void toggleScissors() {
        if (stateScissors == StateScissors.Extended || stateScissors == StateScissors.Custom) retractScissors();
        else extendScissors();
    }
}
