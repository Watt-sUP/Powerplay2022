package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Glisiere {

    public DcMotor motor;
    public DcMotor motor2;
    private int[] positions = {0, 250, 500, 1360, 2240, 3020};

    public Glisiere(HardwareMap hardwareMap) {

        motor = hardwareMap.dcMotor.get(Config.glisiera);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motor2 = hardwareMap.dcMotor.get(Config.glisiera1);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setTargetPosition(0);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setToPosition(int position) {
        motor.setTargetPosition(positions[position]);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (position != 0) motor.setPower(1);
        else motor.setPower(1);

        motor2.setTargetPosition(positions[position]);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(position != 0) motor2.setPower(1);
        else motor2.setPower(1);
    }

    public void setToTicks(int ticks) {
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(1);

        motor2.setTargetPosition(ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(1);
    }

    public void modifyPosition(int ticks) {
        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2);

        motor2.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setPower(0.2);
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public int getTicks() {
        return motor.getCurrentPosition();

        // return motor2.getCurrentPosition();
    }
}
