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
import org.firstinspires.ftc.teamcode.commands.helpers.Cone;

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
    public ConeCommandMidRight(@NonNull Cone cone, ColectareSubsystem colectareSystem, TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            colectareSystem.setScissorsPosition(0.4);
                            turelaSystem.setToTicks(cone.conePos, 0.8);
                        }),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > -300),
                                new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                        )
                ),
                new WaitUntilCommand(() -> turelaSystem.getTicks() + 50 > cone.conePos && glisiereSystem.getTicks() < cone.glisPos + 50),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.coneScissors)),
                new WaitCommand(250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(300),
                new InstantCommand(() -> glisiereSystem.setToTicks(1075), glisiereSystem),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() > cone.glisPos + 400),
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            colectareSystem.retractScissors();
                            turelaSystem.setToTicks(cone.stickPos, 0.45);
                        }),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> glisiereSystem.getTicks() > 400 && turelaSystem.getTicks() < -600),
                                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.stickScissors)),
                                new WaitCommand(200),
                                new InstantCommand(() -> colectareSystem.plastic.turnToAngle(220))
                        )
                ),
                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                new WaitCommand(300),
                new InstantCommand(() -> {
                    colectareSystem.setScissorsPosition(0.5);
                    glisiereSystem.setToTicks(500);
                }),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    colectareSystem.toggleClaw();
                    colectareSystem.plastic.turnToAngle(0);
                }),
                new WaitCommand(100)
        );
        addRequirements(colectareSystem, turelaSystem, glisiereSystem);
    }
}