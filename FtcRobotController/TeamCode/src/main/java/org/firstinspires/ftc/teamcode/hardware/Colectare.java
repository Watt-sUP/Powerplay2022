package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Colectare {
    private final double desfacutPos = 0.5;
    private StateDeget stateDeget;
    public ServoEx deget;

    private enum StateDeget {
        Desfacut,
        Strans
    }

    public Colectare(HardwareMap hardwareMap) {
        stateDeget = StateDeget.Desfacut;
        deget = new SimpleServo(hardwareMap, Config.deget, -360, 360);
        deget.setPosition(desfacutPos);
    }

    public void strange() {
        double stransPos = 0;
        deget.setPosition(stransPos);
        stateDeget = StateDeget.Strans;
    }

    public void desface() {
        deget.setPosition(this.desfacutPos);
        stateDeget = StateDeget.Desfacut;
    }

    public void toggleDeget() {
        if (stateDeget == StateDeget.Desfacut) strange();
        else desface();
    }
}
