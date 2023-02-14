package org.firstinspires.ftc.teamcode.commands;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autonom.AutonomStangaMijloc;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Cone;

/**
 * <p>Helper command for scoring a cone.</p>
 * <p>This command will deposit the cone on the middle junction when the robot is on the left side.</p>
 */
public class ConeCommandMidLeft extends SequentialCommandGroup {

    /**
     * Runs a new command for scoring a cone.
     * @param cone The data about the cone
     * @param colectareSystem The subsystem for the claw and scissors
     * @param turelaSystem The subsystem for the turret
     * @param glisiereSystem The subsystem for the slides
     */
    public ConeCommandMidLeft(@NonNull Cone cone, ColectareSubsystem colectareSystem, TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem) {
        addCommands(
            new InstantCommand(() -> colectareSystem.setScissorsPosition(0.35)),
            new ParallelCommandGroup(
                    new InstantCommand(() -> turelaSystem.setToTicks(cone.conePos)),
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> turelaSystem.getTicks() < 0),
                        new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                    )
            ),
            new WaitUntilCommand(() -> turelaSystem.getTicks() - 50 < cone.conePos && glisiereSystem.getTicks() < cone.glisPos + 50),
            new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.coneScissors)),
            new WaitCommand(250),
            new InstantCommand(colectareSystem::toggleClaw),
            new WaitCommand(300),
            new InstantCommand(() -> glisiereSystem.setToTicks(1450), glisiereSystem),
            new WaitUntilCommand(() -> glisiereSystem.getTicks() > cone.glisPos + 400),
            new ParallelCommandGroup(
                    new InstantCommand(colectareSystem::retractScissors),
                    new InstantCommand(() -> turelaSystem.setToTicks(cone.stickPos, 0.8)),
                    new SequentialCommandGroup(
                            new WaitUntilCommand(() -> turelaSystem.getTicks() > AutonomStangaMijloc.DROP_TICKS),
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
