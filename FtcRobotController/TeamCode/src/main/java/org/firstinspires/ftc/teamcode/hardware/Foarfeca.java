package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Foarfeca {
    private static final double desfacutPos = 0.5;
    private StateFoarfeca stateFoarfeca;
    public ServoEx foarfeca;

        private enum StateFoarfeca {
            Desfacut,
            Strans
        }

        public Foarfeca(HardwareMap hardwareMap) {
            stateFoarfeca = StateFoarfeca.Desfacut;
            foarfeca = new SimpleServo(hardwareMap, Config.foarfeca, 0, 360);
            foarfeca.setPosition(desfacutPos);
        }

        public void strange() {
            double stransPos = 1;
            foarfeca.setPosition(stransPos);
            stateFoarfeca = StateFoarfeca.Strans;
        }

        public void desface() {
            foarfeca.setPosition(desfacutPos);
            stateFoarfeca = StateFoarfeca.Desfacut;
        }

        public void toggleFoarfeca() {
            if (stateFoarfeca == StateFoarfeca.Desfacut) strange();
            else desface();
        }
}
