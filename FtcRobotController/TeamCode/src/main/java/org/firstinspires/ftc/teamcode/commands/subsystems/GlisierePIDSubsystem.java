package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class GlisierePIDSubsystem extends SubsystemBase {

    private final DcMotorEx motor, motor2;
    private boolean use_automations;
    //    public static double kP = 4.5, kI = 0.1, kD = 0;
    public final int[] positions = {0, 300, 775, 1300, 1835};
    private final PIDController pidController = new PIDController(5, 0.1, 0);
    public int position;

    private final ServoEx ghidaj;
    private final Command ghidajDelay;
    private StateGhidaj stateGhidaj;

    private enum StateGhidaj {
        Active,
        Inactive
    }

    public GlisierePIDSubsystem(DcMotorEx motor, DcMotorEx motor2, ServoEx ghidaj) {
        this.motor = motor;
        this.motor2 = motor2;
        this.ghidaj = ghidaj;
        this.use_automations = true;

        this.ghidajDelay = new WaitUntilCommand(() -> motor.getCurrentPosition() >= positions[2])
                .andThen(new InstantCommand(() -> {
                            ghidaj.turnToAngle(190);
                            stateGhidaj = StateGhidaj.Active;
                        })
                );
        closeGhidaj();

        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController.reset();
        pidController.setTolerance(0);
        pidController.setIntegrationBounds(-20000, 20000);
    }

    public GlisierePIDSubsystem(DcMotorEx motor, DcMotorEx motor2, ServoEx ghidaj, Boolean use_automations) {
        this(motor, motor2, ghidaj);
        this.use_automations = use_automations;
    }

    public void setToPosition(int position) {
        this.position = MathUtils.clamp(position, 0, positions.length - 1);
        pidController.setSetPoint(positions[this.position]);

        if (this.position == 4 && use_automations) openGhidaj();
        else if (use_automations) closeGhidaj();

        new RunCommand(
                () -> {
                    double output = pidController.calculate(motor.getCurrentPosition());
                    motor.setVelocity(output);
                    motor2.setVelocity(output);
                }, this
        ).schedule(true);
    }

    public void setToTicks(int ticks) {
        pidController.setSetPoint(ticks);
        if (ticks >= positions[4] && use_automations) openGhidaj();
        else if (use_automations) closeGhidaj();

        new RunCommand(
                () -> {
                    double output = pidController.calculate(motor.getCurrentPosition());
                    motor.setVelocity(output);
                    motor2.setVelocity(output);
                }, this
        ).schedule(true);
    }

    public void modifyByTicks(int ticks) {
        pidController.setSetPoint(motor.getCurrentPosition() + ticks);

        new RunCommand(
                () -> {
                    double output = pidController.calculate(motor.getCurrentPosition());
                    motor.setVelocity(output);
                    motor2.setVelocity(output);
                }, this
        ).schedule(true);
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

    public int getTicks() {
        return motor.getCurrentPosition();
    }
}
