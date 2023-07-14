package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Cone;

/**
 * Helper command for scoring a cone.
 */
public class ConeCommandMidRight extends SequentialCommandGroup {

    /**
     * Runs a new command for scoring a cone.
     *
     * @param cone            The data about the cone
     * @param colectareSystem The subsystem for the claw and scissors
     * @param turelaSystem    The subsystem for the turret
     * @param glisiereSystem  The subsystem for the slides
     */
    public ConeCommandMidRight(@NonNull Cone cone, ColectareSubsystem colectareSystem, TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem, SensorSubsystem sensorSystem) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> colectareSystem.setScissorsPosition(0.4)),
                        new InstantCommand(() -> turelaSystem.setToTicks(cone.conePos, 0.7)),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > 150),
                                new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                        )
                ),
                new WaitUntilCommand(() -> turelaSystem.getTicks() + 50 > cone.conePos && glisiereSystem.getTicks() < cone.glisPos + 50),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.coneScissors)),
                new WaitCommand(250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new InstantCommand(() -> glisiereSystem.setToPosition(3), glisiereSystem),
                        new InstantCommand(glisiereSystem::raiseUnghi)
                ),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() > cone.glisPos + (400 * 0.9)),
                new ParallelCommandGroup(
                        new InstantCommand(colectareSystem::retractScissors),
                        new InstantCommand(() -> turelaSystem.setToTicks(cone.stickPos, 0.9)),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                                new ScanPoleCommand(turelaSystem, sensorSystem, new Pair<>(-475, -650), 50.0),
                                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.stickScissors)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> glisiereSystem.setToPosition(2)),
                                        new InstantCommand(glisiereSystem::lowerUnghi)
                                )
                        )
                ),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() < 800),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(100)
        );
        addRequirements(colectareSystem, turelaSystem, glisiereSystem);
    }
}
