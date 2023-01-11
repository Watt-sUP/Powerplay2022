package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

@com.acmerobotics.dashboard.config.Config
public class GlisiereBeta {

    // TODO: Tune the Position Coefficient
    public static volatile double kP = 0.1;
    private Goal goal;
    private final HashMap<Goal, Integer> goal_values;
    private final int[] positions = {0, 300, 944, 1541, 2250};

    private enum Goal {
        RAISE,
        LOWER
    }

    Motor gli, gli2;
    MotorGroup glisiere;

    public GlisiereBeta(HardwareMap hardwareMap) {
        gli = new Motor(hardwareMap, Config.glisiera, Motor.GoBILDA.RPM_435);
        gli2 = new Motor(hardwareMap, Config.glisiera1, Motor.GoBILDA.RPM_435);
        gli2.setInverted(true);
        gli2.encoder = gli.encoder;

        glisiere = new MotorGroup(gli, gli2);
        glisiere.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        glisiere.setRunMode(Motor.RunMode.PositionControl);
        glisiere.setPositionCoefficient(kP);

        goal_values = new HashMap<>();
        goal_values.put(Goal.RAISE, 1540);
        goal_values.put(Goal.LOWER, 300);
    }

    public void tuner(@NonNull GamepadEx gamepad, @Nullable Telemetry telemetry) throws InterruptedException {
        goal = Goal.RAISE;
        int goal_ticks = goal_values.getOrDefault(goal, 700);
        glisiere.set(0.0);
        glisiere.setTargetPosition(goal_ticks);
        glisiere.setPositionTolerance(10);

        while (!gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {

            if (kP != glisiere.getPositionCoefficient())
                glisiere.setPositionCoefficient(kP);

            if (telemetry != null) {
                telemetry.addData("Motor reversed:", gli2.getInverted());
                telemetry.addData("Current kP:", glisiere.getPositionCoefficient());
                telemetry.addData("Current encoder position:", glisiere.getCurrentPosition());
                telemetry.addData("Current goal:", goal_ticks);

                telemetry.update();
            }

            if (!glisiere.atTargetPosition())
                glisiere.set(1.0);
            else {
                glisiere.stopMotor();
                toggleGoal();
                sleep(1250);
                goal_ticks = goal_values.getOrDefault(goal, 700);
                glisiere.setTargetPosition(goal_ticks);
                glisiere.set(0.0);
                glisiere.setPositionTolerance(10);
            }
        }

    }

    public void toggleGoal() {
        this.goal = (this.goal == Goal.RAISE || this.goal == Goal.LOWER) ? Goal.LOWER : Goal.RAISE;
    }
}
