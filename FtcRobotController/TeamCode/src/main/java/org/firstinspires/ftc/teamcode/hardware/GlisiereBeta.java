package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GlisiereBeta {
    private final int[] positions = {0, 300, 500, 944, 1541, 2250};

    Motor gli, gli2;
    MotorGroup glisiere;

    public GlisiereBeta(HardwareMap hardwareMap)
    {
        gli = new Motor(hardwareMap, Config.glisiera, Motor.GoBILDA.RPM_435);
        gli2 = new Motor(hardwareMap, Config.glisiera1, Motor.GoBILDA.RPM_435);
        gli2.setInverted(true);
        gli2.encoder = gli.encoder;
        glisiere = new MotorGroup(gli, gli2);
        glisiere.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
}
