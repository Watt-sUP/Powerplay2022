package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mugurel {

    public Glisiere glisiere;
    public Turela turela;
    public DeadWheels wheels;
    public Colectare deget;

    public Mugurel(HardwareMap hm) {
        glisiere = new Glisiere(hm);
        turela = new Turela(hm);
        wheels = new DeadWheels(hm);
    }
}