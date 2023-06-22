package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;

public class ScanPoleCommand extends CommandBase {

    private final TurelaSubsystem turela;
    private final SensorSubsystem sensor;
    private final Integer startPos;
    private Double lastDistance = null;

    public ScanPoleCommand(TurelaSubsystem turela, SensorSubsystem sensor, Integer startPos) {
        this.sensor = sensor;
        this.turela = turela;
        this.startPos = startPos;

        addRequirements(turela, sensor);
    }

    @Override
    public void initialize() {
        sensor.setColorThreshold(
                new Pair<>(320f, 350f),
                new Pair<>(0.45f, 1f),
                new Pair<>(0.1f, 1f)
        );
        lastDistance = sensor.getDistance();
        turela.setToTicks(startPos);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
