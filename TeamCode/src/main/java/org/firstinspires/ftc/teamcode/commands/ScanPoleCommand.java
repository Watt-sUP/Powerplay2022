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
    public static int TICK_CHANGE = 25;
    private Integer target_ticks;
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
        this.target_ticks = turela.getTicks();
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
        int ticks = turela.getTicks();

        if (distance > lastDistance && lastDistance < limit)
            return true;

        if (distance < lastDistance)
            target_ticks = ticks;

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            turela.setToTicks(target_ticks, 0.33);
        }
    }
}
