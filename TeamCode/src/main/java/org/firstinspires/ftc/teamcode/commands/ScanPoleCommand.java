package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;

@Config
public class ScanPoleCommand extends CommandBase {

    private final TurelaSubsystem turela;
    private final SensorSubsystem sensor;
    private Integer targetTicks;
    private Double lastDistance = 1000.0, limit;
    public static int TICK_CHANGE = 50;

    public enum Direction {
        LEFT,
        RIGHT
    }
    private final Direction direction;

    public ScanPoleCommand(TurelaSubsystem turela, SensorSubsystem sensor, Integer startPos, Direction direction, Double limit) {
        this.turela = turela;
        this.sensor = sensor;
        this.targetTicks = startPos;
        this.direction = direction;
        this.limit = limit;

        turela.setToTicks(startPos);
        addRequirements(turela, sensor);
    }

//    @Override
//    public void initialize() {
//        turela.setToTicks(targetTicks);
//    }

    @Override
    public void execute() {

        if (sensor.getDistance() > lastDistance) {
            return;
        }

        lastDistance = sensor.getDistance();
        targetTicks = turela.getTicks();
        turela.modifyTicks((direction == Direction.LEFT) ? TICK_CHANGE : -TICK_CHANGE, 0.5);
    }

    @Override
    public boolean isFinished() {
        double distance = sensor.getDistance();
        return distance > lastDistance && lastDistance < 75.0;
    }

    @Override
    public void end(boolean interrupted) {
        turela.setToTicks(targetTicks);
    }
}
