package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class TurelaBeta {

    public Motor turela;
    public HashMap<Goal, Integer> goal_values;

    public static enum Goal {
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
