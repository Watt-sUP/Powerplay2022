package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.helpers.Cone;
import org.firstinspires.ftc.teamcode.commands.helpers.Location;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;

import java.util.HashMap;

/**
 * <p>General command for scoring a cone when the robot is on the right side.</p>
 * <p>Available depositing locations are the middle and high junctions.</p>
 */
@Config
public class ConeCommandRight extends SequentialCommandGroup {

    public static Location highJunction = new Location(0.51, -1900, 1525),
            midJunction = new Location(0.53, -1400, 1125),
            stack = new Location(0.52, -15, 500);

    /**
     * Runs a new command for scoring a cone.
     * @param cone The cone object containing the target junction and its slider position.
     * @param colectareSystem The collection subsystem.
     * @param turelaSystem The turret subsystem.
     * @param glisiereSystem The slider subsystem.
     */
    public ConeCommandRight(Cone cone, ColectareSubsystem colectareSystem, TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            colectareSystem.setScissorsPosition(0.4);
                            turelaSystem.setToTicks(stack.turretPosition, 0.9);
                        }),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > -300),
                                new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                        )
                ),
                new WaitUntilCommand(() -> turelaSystem.getTicks() + 50 > stack.turretPosition && glisiereSystem.getTicks() < cone.glisPos + 50),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(stack.scissorsPosition)),
                new WaitCommand(250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(300),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Cone.Junctions.High, new InstantCommand(() -> glisiereSystem.setToTicks(highJunction.sliderPosition)));
                            put(Cone.Junctions.Middle, new InstantCommand(() -> glisiereSystem.setToTicks(midJunction.sliderPosition)));
                        }},
                        () -> cone.targetJunction
                ),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() > cone.glisPos + 400),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Cone.Junctions.High, new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        colectareSystem.retractScissors();
                                        turelaSystem.setToTicks(highJunction.turretPosition + 75);
                                    }),
                                    new WaitUntilCommand(() -> turelaSystem.getTicks() < highJunction.turretPosition * 0.4),
                                    new InstantCommand(() -> colectareSystem.setScissorsPosition(highJunction.scissorsPosition)),
                                    new WaitCommand(200),
                                    new InstantCommand(() -> {
                                        colectareSystem.plastic.turnToAngle(220);
                                        turelaSystem.setToTicks(highJunction.turretPosition, 0.33);
                                    })
                            ));
                            put(Cone.Junctions.Middle, new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        colectareSystem.retractScissors();
                                        turelaSystem.setToTicks(midJunction.turretPosition, 0.45);
                                    }),
                                    new WaitUntilCommand(() -> glisiereSystem.getTicks() > 400 && turelaSystem.getTicks() < -600),
                                    new InstantCommand(() -> colectareSystem.setScissorsPosition(midJunction.scissorsPosition)),
                                    new WaitCommand(200),
                                    new InstantCommand(() -> colectareSystem.plastic.turnToAngle(220))
                            ));
                        }},
                        () -> cone.targetJunction
                ),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Cone.Junctions.High, new WaitUntilCommand(() -> turelaSystem.getTicks() < highJunction.turretPosition + 50));
                            put(Cone.Junctions.Middle, new WaitUntilCommand(() -> turelaSystem.getTicks() < midJunction.turretPosition + 50));
                        }},
                        () -> cone.targetJunction
                ),
                new WaitCommand(300),
                new InstantCommand(() -> {
                    colectareSystem.setScissorsPosition(0.5);
                    glisiereSystem.setToTicks(stack.sliderPosition);
                }),
                new WaitCommand(250),
                new InstantCommand(() -> {
                    colectareSystem.toggleClaw();
                    colectareSystem.plastic.turnToAngle(0);
                })
        );
        addRequirements(colectareSystem, turelaSystem, glisiereSystem);
    }
}
