package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turela {

    public DcMotor motortur;
    private int[] POSITIONS = {0, -1000, -2030, 1000, 0};

    public Turela(HardwareMap hardwareMap) {
        motortur = hardwareMap.dcMotor.get(Config.turela);
        motortur.setDirection(DcMotorSimple.Direction.FORWARD);
        motortur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motortur.setTargetPosition(0);
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setToPosition(int position) {
        motortur.setTargetPosition(POSITIONS[position]);
        motortur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (position != 0) motortur.setPower(1);
        else motortur.setPower(1);
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
