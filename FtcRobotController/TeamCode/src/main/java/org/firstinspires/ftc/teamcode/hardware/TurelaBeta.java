package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class TurelaBeta {

    public Motor turela;
    public Map<Goal, Integer> goal_values;

    public enum Goal {
        LEFT,
        RIGHT
    }

    public TurelaBeta(HardwareMap hardwareMap) {
        turela = new Motor(hardwareMap, Config.turela, Motor.GoBILDA.RPM_1150);
        turela.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turela.setRunMode(Motor.RunMode.PositionControl);
        turela.setPositionCoefficient(0.05);

        goal_values = new HashMap<>();
        goal_values.put(Goal.LEFT, 1000);
        goal_values.put(Goal.RIGHT, -1000);
    }
}
