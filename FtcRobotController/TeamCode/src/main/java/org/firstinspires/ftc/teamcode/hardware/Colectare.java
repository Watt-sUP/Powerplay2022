package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Colectare {
    private final double desfacutPos = 0.05;
    private StateDeget stateDeget;
    public Servo deget;

    private enum StateDeget {
        Desfacut,
        Strans
    }

    public Colectare(HardwareMap hardwareMap) {
        stateDeget = StateDeget.Desfacut;
        deget = hardwareMap.servo.get(Config.deget);
        deget.setPosition(desfacutPos);
    }

    public void strange() {
        double stransPos = 0;
        deget.setPosition(stransPos);
        stateDeget = StateDeget.Strans;
    }

    public void desface() {
        deget.setPosition(desfacutPos);
        stateDeget = StateDeget.Desfacut;
    }

    public void toggleDeget() {
        if (stateDeget == StateDeget.Desfacut) strange();
        else desface();
    }
}
