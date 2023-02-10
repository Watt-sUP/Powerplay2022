package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Cone;

/**
 * Helper command for scoring a cone.
 */
public class ConeCommand extends SequentialCommandGroup {
    /**
     * Runs a new command for scoring a cone.
     * @param cone The data about the cone
     * @param colectareSystem The subsystem for the claw and scissors
     * @param turelaSystem The subsystem for the turret
     * @param glisiereSystem The subsystem for the slides
     */
    public ConeCommand(@NonNull Cone cone, ColectareSubsystem colectareSystem, TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem) {
        addCommands(
            new InstantCommand(() -> colectareSystem.setScissorsPosition(0.35)),
            new ParallelCommandGroup(
                    new InstantCommand(() -> turelaSystem.setToTicks(cone.conePos)),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> turelaSystem.getTicks() < 0),
                        new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                    )
            ),
            new WaitCommand(900),
            new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.coneScissors)),
            new WaitCommand(300),
            new InstantCommand(colectareSystem::toggleClaw),
            new WaitCommand(300),
            new InstantCommand(() -> glisiereSystem.setToPosition(3), glisiereSystem),
            new WaitUntilCommand(() -> glisiereSystem.getTicks() > 1000),
            new ParallelCommandGroup(
                    new InstantCommand(() -> turelaSystem.setToTicks(cone.stickPos, 0.8)),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.stickScissors)),
                            new WaitUntilCommand(() -> turelaSystem.getTicks() > 400),
                            new InstantCommand(() -> glisiereSystem.setToPosition(2))
                    )
            ),
            new WaitCommand(200),
            new InstantCommand(colectareSystem::toggleClaw),
            new WaitCommand(100)
        );
        addRequirements(colectareSystem, turelaSystem, glisiereSystem);
    }
}