package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;

@Config
public class ScanPoleCommand extends CommandBase {

    private final TurelaSubsystem turela;
    private final SensorSubsystem sensor;
    private Double lastDistance;
    private final Double limit;
    public static int TICK_CHANGE = 20;
    public static double POWER = 0.33;

    public enum Direction {
        LEFT,
        RIGHT
    }
    private final Direction direction;

    public ScanPoleCommand(TurelaSubsystem turela, SensorSubsystem sensor, Direction direction, Double limit) {
        this.turela = turela;
        this.sensor = sensor;
        this.direction = direction;
        this.lastDistance = Double.MAX_VALUE;
        this.limit = limit;

        addRequirements(turela, sensor);
    }

    @Override
    public void execute() {
        lastDistance = sensor.getDistance();
        turela.modifyTicks((direction == Direction.LEFT) ? TICK_CHANGE : -TICK_CHANGE, POWER);
    }

    @Override
    public boolean isFinished() {
        double distance = sensor.getDistance();
        return distance > lastDistance && lastDistance < limit;
    }
}
