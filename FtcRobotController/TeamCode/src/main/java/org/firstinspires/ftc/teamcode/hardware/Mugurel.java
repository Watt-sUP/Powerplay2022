package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mugurel {

    public Glisiere glisiera;
    public Turela turela;
    public Colectare deget;
    public DriveMotors driveMotors;

    public Mugurel(HardwareMap hm) {
        glisiera = new Glisiere(hm);
        turela = new Turela(hm);
        deget = new Colectare(hm);
        driveMotors = new DriveMotors(hm);
    }

    public void shutdown_system_motors()
    {
        glisiera.motor.setPower(0);
        glisiera.motor2.setPower(0);
        turela.motortur.setPower(0);
    }
}