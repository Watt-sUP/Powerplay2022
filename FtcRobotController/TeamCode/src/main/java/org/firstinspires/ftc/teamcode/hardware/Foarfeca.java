package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Foarfeca {
    private static final double stransPos = 0.5;
    private StateFoarfeca stateFoarfeca;
    public ServoEx foarfeca;

        private enum StateFoarfeca {
            Strans,
            Intins,
            Custom
        }

        public Foarfeca(HardwareMap hardwareMap) {
            foarfeca = new SimpleServo(hardwareMap, Config.foarfeca, 0, 360);
            foarfeca.setPosition(stransPos);
            stateFoarfeca = StateFoarfeca.Strans;
        }

        public void setToPosition(double position) {
            foarfeca.setPosition(position);
            stateFoarfeca = StateFoarfeca.Custom;
        }

        public void intinde() {
            double intinsPos = 1;
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
