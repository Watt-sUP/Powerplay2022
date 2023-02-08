package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Colectare {
    private final double openPos = 0.5;
    private StateClaw stateClaw;
    public ServoEx claw;

    private enum StateClaw {
        Opened,
        Closed
    }

    public Colectare(HardwareMap hardwareMap) {
        stateClaw = StateClaw.Opened;
        claw = new SimpleServo(hardwareMap, Config.claw, -360, 360);
        claw.setInverted(false);
        claw.setPosition(openPos);
    }

    public void close() {
        double closePos = 0;
        claw.setPosition(closePos);
        stateClaw = StateClaw.Closed;
    }

    public void open() {
        claw.setPosition(openPos);
        stateClaw = StateClaw.Opened;
    }

    public void toggleClaw() {
        if (stateClaw == StateClaw.Opened) close();
        else open();
    }
}
