package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Hashtable;

public class Turela {

    public DcMotor motortur;
    private Hashtable<Position, Integer> pos_dict;

    public enum Position {
        FRONT,
        BACK,
        LEFT,
        RIGHT
    }

    public Turela(HardwareMap hardwareMap) {
        motortur = hardwareMap.dcMotor.get(Config.turela);
        motortur.setDirection(DcMotorSimple.Direction.FORWARD);
        motortur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pos_dict = new Hashtable<>();
        pos_dict.put(Position.FRONT, 0);
        pos_dict.put(Position.BACK, -2030);
        pos_dict.put(Position.LEFT, 1000);
        pos_dict.put(Position.RIGHT, -1000);
    }

    public void setToPosition(Position position) {
        motortur.setTargetPosition(pos_dict.get(position));
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortur.setPower(1);
    }

    public void setToTicks(int ticks) {
        motortur.setTargetPosition(ticks);
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortur.setPower(1);
    }

    public void modifyPosition(int ticks) {
        motortur.setTargetPosition(motortur.getCurrentPosition() + ticks);
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motortur.setPower(1);
    }

    public double getPosition() {
        return motortur.getCurrentPosition();
    }

    public int getTicks() {
        return motortur.getCurrentPosition();
    }
}
