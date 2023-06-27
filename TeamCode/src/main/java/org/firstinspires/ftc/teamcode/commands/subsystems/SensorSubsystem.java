package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Command-based subsystem for the color and proximity recognition sensors.
 */
public class SensorSubsystem extends SubsystemBase {
    private final SensorRevTOFDistance distanceSensor;

    /**
     * Creates a new instance of the subsystem.
     *
     * @param hardwareMap HardwareMap object for importing the sensors
     * @param sensor      String representing the name of the sensor
     */
    public SensorSubsystem(HardwareMap hardwareMap, String sensor) {
        distanceSensor = new SensorRevTOFDistance(hardwareMap, sensor);
    }

    /**
     * Gets the distance detected by the proximity sensor.
     * @return Double representing the distance in centimeters
     */
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}