package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Math.abs;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;

@Config
public class ScanPoleCommand extends CommandBase {

    private final TurelaSubsystem turela;
    private final SensorSubsystem sensor;
    private Double minDistance;
    private final Double limit;
    public static int TICK_CHANGE = 50;
    private Integer target_ticks;
    public static double POWER = 0.33;
    private final Pair<Integer, Integer> interval;

    public ScanPoleCommand(TurelaSubsystem turela, SensorSubsystem sensor, Pair<Integer, Integer> interval, Double limit) {
        this.turela = turela;
        this.sensor = sensor;
        this.interval = interval;
        this.minDistance = Double.MAX_VALUE;
        this.target_ticks = interval.first;
        this.limit = limit;

        addRequirements(turela, sensor);
    }

    @Override
    public void execute() {
        turela.modifyTicks((interval.first < interval.second) ? TICK_CHANGE : -TICK_CHANGE, POWER);
    }

    @Override
    public boolean isFinished() {
        double distance = sensor.getDistance();
        int ticks = turela.getTicks();

        if (abs(ticks) > abs(interval.second) && minDistance < limit)
            return true;

        if (distance < minDistance) {
            minDistance = distance;
            target_ticks = ticks;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            turela.setToTicks(target_ticks, 0.2);
        }
    }
}
