package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Colectare {
    private static double stransPos = 0, desfacutPos = 0.05, desfacutMorePos = 0.65;
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
        deget.setPosition(stransPos);
        stateDeget = StateDeget.Strans;
    }

    public void desface() {
        deget.setPosition(desfacutPos);
        stateDeget = StateDeget.Desfacut;
    }

    public void toggleDeget() {
        if(stateDeget == StateDeget.Desfacut) strange();
        else if (stateDeget != StateDeget.Desfacut) desface();
    }

    // Desfacut mai mult
    public void desfaceMore() {
        deget.setPosition(desfacutMorePos);
        stateDeget = StateDeget.Desfacut;
    }

    public void toggleDegetMore() {
        if(stateDeget == StateDeget.Desfacut) strange();
        else if (stateDeget != StateDeget.Desfacut) desfaceMore();
    }


}
