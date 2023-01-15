package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
    private double y, x, rx;
    private double powerLimit = 1.0;
    private MecanumDrive drive;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack, GamepadEx gamepad) {
        this.y = gamepad.getLeftY();
        this.x = gamepad.getLeftX();
        this.rx = gamepad.getRightX();

        this.drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    public void drive() {
        drive.driveRobotCentric(x, y, rx);
    }

    public void setPowerLimit(double limit) {
        this.powerLimit = limit;
        drive.setMaxSpeed(MathUtils.clamp(limit, -1.0, 1.0));
    }

    public double getPowerLimit() {
        return this.powerLimit;
    }

    public void getPowerLimit(Telemetry telemetry) {
        telemetry.addData("Power Limit:", this.powerLimit);
    }
}
