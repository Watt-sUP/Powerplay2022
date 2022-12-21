package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mugurel {

    public Glisiere glisiera;
    public Turela turela;
    //public DeadWheels wheels;
    public Colectare deget;


    public Mugurel(HardwareMap hm) {
        glisiera = new Glisiere(hm);
        turela = new Turela(hm);
        deget = new Colectare(hm);
        //  wheels = new DeadWheels(hm);
    }

    public void shutdown_motors()
    {
        glisiera.motor.setPower(0);
        glisiera.motor2.setPower(0);
        turela.motortur.setPower(0);
    }

    public void glisiera(boolean dpad_up, boolean dpad_down, boolean b, boolean right_bumper, boolean left_bumper) {
    }

    public void turela(boolean dpad_right, boolean dpad_left, boolean a, boolean x, boolean y) {
    }
}