package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Foarfeca {
        private static double stransPos = 0, desfacutPos = 0.6;
        private StateFoarfeca stateFoarfeca;
        public Servo foarfeca;

        private enum StateFoarfeca {
            Desfacut,
            Strans
        }

        public Foarfeca(HardwareMap hardwareMap) {
            stateFoarfeca = StateFoarfeca.Desfacut;
            foarfeca = hardwareMap.servo.get(Config.foarfeca);
            foarfeca.setPosition(desfacutPos);
        }

        public void strange() {
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
