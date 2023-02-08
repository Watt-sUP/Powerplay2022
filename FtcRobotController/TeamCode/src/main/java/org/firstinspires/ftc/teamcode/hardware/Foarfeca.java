package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Foarfeca {
    private final double stransPos;
    private StateFoarfeca stateFoarfeca;
    public ServoEx foarfeca;

    private enum StateFoarfeca {
        Strans,
        Intins,
        Custom
    }

    public Foarfeca(HardwareMap hardwareMap, double init_pos) {
        foarfeca = new SimpleServo(hardwareMap, Config.foarfeca, 0, 360);
        foarfeca.setInverted(true);
        foarfeca.setPosition(init_pos);
        stransPos = init_pos;
        stateFoarfeca = StateFoarfeca.Strans;
    }

    public Foarfeca(HardwareMap hardwareMap) {
        this(hardwareMap, 0.2);
    }

    public void setToPosition(double position) {
        foarfeca.setPosition(position);
        stateFoarfeca = StateFoarfeca.Custom;
    }

    public void intinde() {
        double intinsPos = 0.75;
        foarfeca.setPosition(intinsPos);
        stateFoarfeca = StateFoarfeca.Intins;
    }

    public void strange() {
        foarfeca.setPosition(stransPos);
        stateFoarfeca = StateFoarfeca.Strans;
    }

    public void toggleFoarfeca() {
        if (stateFoarfeca == StateFoarfeca.Intins || stateFoarfeca == StateFoarfeca.Custom) strange();
        else intinde();
    }
}
