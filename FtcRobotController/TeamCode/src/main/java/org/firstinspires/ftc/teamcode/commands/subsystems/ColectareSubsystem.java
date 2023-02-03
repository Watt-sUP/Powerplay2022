package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class ColectareSubsystem extends SubsystemBase {
    private ServoEx gheara, foarfeca;
    private StateGheara stateGheara;
    private StateFoarfeca stateFoarfeca;

    private enum StateGheara {
        Desfacut,
        Strans
    }
    private enum StateFoarfeca {
        Intins,
        Strans
    }

    public ColectareSubsystem(ServoEx gheara, ServoEx foarfeca) {
        this.gheara = gheara;
        this.foarfeca = foarfeca;
        gheara.setPosition(0.5);
        foarfeca.setPosition(0.5);
        stateGheara = StateGheara.Desfacut;
        stateFoarfeca = StateFoarfeca.Strans;
    }

    public void strangeGheara() {
        double stransPos = 1;
        gheara.setPosition(stransPos);
        stateGheara = StateGheara.Strans;
    }

    public void desfaceGheara() {
        gheara.setPosition(0.5);
        stateGheara = StateGheara.Desfacut;
    }

    public void toggleGheara() {
        if (stateGheara == StateGheara.Desfacut) strangeGheara();
        else desfaceGheara();
    }

    public void intindeFoarfeca() {
        double intinsPos = 1;
        foarfeca.setPosition(intinsPos);
        stateFoarfeca = StateFoarfeca.Intins;
    }

    public void strangeFoarfeca() {
        foarfeca.setPosition(0.5);
        stateFoarfeca = StateFoarfeca.Strans;
    }

    public void toggleFoarfeca() {
        if (stateFoarfeca == StateFoarfeca.Intins) strangeFoarfeca();
        else intindeFoarfeca();
    }
}
