package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Hashtable;

public class TurelaSubsystem extends SubsystemBase {
    private DcMotor motor;
    private final Hashtable<Direction, Integer> posDict;

    public TurelaSubsystem(Motor turelaMotor) {
        this.motor = turelaMotor.motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.posDict = new Hashtable<>();

        posDict.put(Direction.LEFT, 1000);
        posDict.put(Direction.RIGHT, -1000);
        posDict.put(Direction.FORWARD, 0);
        posDict.put(Direction.BACKWARDS, -2030);
    }

    public void setToPosition(Direction direction, double power) {
        motor.setTargetPosition(posDict.get(direction));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void setToTicks(int ticks, double power) {
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void modifyByTicks(int ticks, double power) {
        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public void setToPosition(Direction direction) {
        setToPosition(direction, 1.0);
    }
    public void setToTicks(int ticks) {
        setToTicks(ticks, 1.0);
    }
    public void modifyByTicks(int ticks) {
        modifyByTicks(ticks, 0.7);
    }
}
