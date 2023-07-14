package org.firstinspires.ftc.teamcode.commands;

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

public class ConeLeftSensorCommand extends SequentialCommandGroup {
    public enum Poles {
        MIDDLE,
        DANGER_HIGH,
        SAFE_HIGH
    }

    public ConeLeftSensorCommand(Cone cone, Poles target_pole, ColectareSubsystem colectareSystem,
                                 TurelaSubsystem turelaSystem, GlisiereSubsystem glisiereSystem, SensorSubsystem sensorSystem) {
        addCommands(
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new InstantCommand(colectareSystem::retractScissors),
                        new InstantCommand(glisiereSystem::lowerUnghi),
                        new InstantCommand(() -> turelaSystem.setToTicks(cone.conePos)),
                        new InstantCommand(() -> glisiereSystem.setToTicks(cone.glisPos))
                ),
                new WaitUntilCommand(() -> !turelaSystem.isBusy() && !glisiereSystem.isBusy()),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.coneScissors)),
                new WaitCommand(400),
                new InstantCommand(colectareSystem::closeClaw),
                new WaitCommand(300),
                new InstantCommand(() -> glisiereSystem.setToPosition(4)),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() > cone.glisPos + 400),
                new ParallelCommandGroup(
                        new InstantCommand(glisiereSystem::raiseUnghi),
                        new InstantCommand(colectareSystem::retractScissors)
                ),
                new WaitCommand(400),
                new InstantCommand(() -> turelaSystem.setToTicks(cone.stickPos, 0.66)),
                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(0.67)),
                //new ScanPoleCommand(turelaSystem, sensorSystem, ScanPoleCommand.Direction.LEFT, 50.0),
                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(cone.stickScissors))
        );
        addRequirements(colectareSystem, turelaSystem, glisiereSystem, sensorSystem);
    }
}
