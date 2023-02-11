package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.MathUtils;

public class DriveSubsystem extends SubsystemBase {
    private double powerLimit = 1.0;
    private final MecanumDrive drive;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack) {
        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    public void drive(double fwd, double st, double rot) {
        drive.driveRobotCentric(st, fwd, rot, true);
    }

    public double getPowerLimit() {
        return powerLimit;
    }

    public void setPowerLimit(double limit) {
        powerLimit = limit;
        drive.setMaxSpeed(MathUtils.clamp(limit, -1.0, 1.0));
    }
}
