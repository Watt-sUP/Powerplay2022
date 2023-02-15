package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonom.AutonomDreaptaSus;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Cone;

// TODO: Add javadocs and finish the command
public class ConeCommandHighRight extends SequentialCommandGroup {

    public ConeCommandHighRight(@NonNull Cone cone, ColectareSubsystem colectareSystem, TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> colectareSystem.setScissorsPosition(0.35)),
                        new InstantCommand(() -> turelaSystem.setToTicks(cone.conePos)),

                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > 0),
                                new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                        )
                ),
                new WaitUntilCommand(() -> turelaSystem.getTicks() + 50 > cone.conePos && glisiereSystem.getTicks() < cone.glisPos + 50),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.coneScissors)),
                new WaitCommand(250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(300),
                new InstantCommand(() -> glisiereSystem.setToTicks(1950), glisiereSystem),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() > cone.glisPos + 400),
                new InstantCommand(colectareSystem::retractScissors),
                new ParallelCommandGroup(
                        new InstantCommand(() -> turelaSystem.setToTicks(cone.stickPos, 0.8)),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() < AutonomDreaptaSus.DROP_TICKS),
                                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.stickScissors)),
                                new WaitCommand(150),
                                new InstantCommand(() -> glisiereSystem.setToPosition(2))
                        )
                ),
                new WaitCommand(250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(100)
        );
        addRequirements(colectareSystem, turelaSystem, glisiereSystem);
    }
}
