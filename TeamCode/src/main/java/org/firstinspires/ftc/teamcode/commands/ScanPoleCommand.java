package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;

public class ScanPoleCommand extends CommandBase {

    private final TurelaSubsystem turela;
    private final SensorSubsystem sensor;
    private final Integer startPos;
    private Integer targetTicks = null;
    private Double lastDistance = Double.MAX_VALUE;

    public enum Direction {
        LEFT,
        RIGHT
    }
    private final Direction direction;

    public ScanPoleCommand(TurelaSubsystem turela, SensorSubsystem sensor, Integer startPos, Direction direction) {
        this.turela = turela;
        this.sensor = sensor;
        this.startPos = startPos;
        this.direction = direction;

        addRequirements(turela, sensor);
    }

    public ScanPoleCommand(TurelaSubsystem turela, SensorSubsystem sensor, Integer startPos) {
        this(turela, sensor, startPos, Direction.LEFT);
    }

    @Override
    public void initialize() {
        sensor.setColorThreshold(
                new Pair<>(320f, 350f),
                new Pair<>(0.45f, 1f),
                new Pair<>(0.1f, 1f)
        );
        turela.setToTicks(startPos);
    }

    @Override
    public void execute() {
        if (turela.isBusy()) {
            return;
        }

        if (sensor.getDistance() > lastDistance) {
            return;
        }

        lastDistance = sensor.getDistance();
        targetTicks = turela.getTicks();
        turela.modifyTicks((direction == Direction.LEFT) ? 25 : -25, 0.5);
    }

    @Override
    public boolean isFinished() {
        return !turela.isBusy() && sensor.getDistance() > lastDistance;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted && targetTicks != null) {
            turela.setToTicks(targetTicks);
        }
        else {
            turela.setToTicks(startPos);
        }
    }
}
