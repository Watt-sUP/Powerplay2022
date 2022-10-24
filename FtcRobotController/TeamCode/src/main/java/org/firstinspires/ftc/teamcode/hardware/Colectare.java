package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Colectare {

    public Servo servo;
    private double departe = 0.99, aproape = 0.05;
    private static double stransPos = 0.25, desfacutPos = 0.53, desfacutMorePos = 0.65;
    private State state;
    private StateDeget stateDeget;
    public Servo deget;

    private enum State {

        Departe,
        Aproape,
    }

    private enum StateDeget {

        Desfacut,
        Strans,
    }

    public Colectare(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get(Config.sersus);
        state = State.Aproape;
        servo.setPosition(aproape);
        stateDeget = StateDeget.Desfacut;
        deget = hardwareMap.servo.get(Config.deget);
        deget.setPosition(desfacutPos);
    }

    public double getServoPosition() {
        return servo.getPosition();
    }

    public void inchis() {
        servo.setPosition(aproape);
        state = State.Aproape;
    }

    public void deschis() {
        servo.setPosition(departe);
        state = State.Departe;
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
        else desface();
    }

    public void toggleDegetMore() {
        if(stateDeget == StateDeget.Desfacut) strange();
        else desfaceMore();
    }

    public void toggleCupa() {
        if(state == State.Aproape) departe();
        else inchis();
    }

    public void desfaceMore() {
        deget.setPosition(desfacutMorePos);
        stateDeget = StateDeget.Desfacut;
    }


}
