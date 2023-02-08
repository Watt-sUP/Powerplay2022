package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * <p>Class compiling all of the robot's subsystems for easier use.</p>
 * <p>Components include the drivetrain motors, sliders and turret.</p>
 */
public class Mugurel {

    public Glisiere glisiera;
    public Turela turela;
    public Colectare claw;
    public DriveMotors driveMotors;
    public Foarfeca foarfeca;

    /**
     * Creates a new robot instance
     * @param hm The hardwareMap necessary to load the components
     */
    public Mugurel(HardwareMap hm) {
        glisiera = new Glisiere(hm);
        turela = new Turela(hm);
        claw = new Colectare(hm);
        driveMotors = new DriveMotors(hm);
        foarfeca = new Foarfeca(hm, 0.3);
    }

    /**
     * Shuts down the motors of the systems to avoid errors
     */
    public void shutdown_system_motors()
    {
        glisiera.motor.setPower(0);
        glisiera.motor2.setPower(0);
        turela.motortur.setPower(0);
    }
}