package org.firstinspires.ftc.teamcode.commands.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

import javax.annotation.Nullable;

/**
 * Command-based subsystem for the color and proximity recognition sensors.
 */
public class SensorSubsystem extends SubsystemBase {
    private final SensorColor colorSensor;
    private final DistanceSensor proximitySensor;

    /**
     * Creates a new instance of the subsystem.
     *
     * @param hardwareMap HardwareMap object for importing the sensors
     * @param sensor      String representing the name of the sensor
     */
    public SensorSubsystem(HardwareMap hardwareMap, String sensor) {
        colorSensor = new SensorColor(hardwareMap, sensor);
        proximitySensor = hardwareMap.get(DistanceSensor.class, sensor);
    }

    /**
     * Gets the distance detected by the proximity sensor.
     * @return Double representing the distance in centimeters
     */
    public double getDistance() {
        return proximitySensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Gets a map of the RGB values detected by the color sensor.
     * @param telemetry Telemetry object to display the values on the driver station (optional)
     * @return Map of the RGB values (Keys: Red, Green, Blue)
     */
    public Map<String, Integer> getRGB(@Nullable Telemetry telemetry) {
        Map<String, Integer> rgb = new HashMap<>();

        rgb.put("Red", colorSensor.red());
        rgb.put("Green", colorSensor.green());
        rgb.put("Blue", colorSensor.blue());

        if (telemetry != null) {
            for (Map.Entry<String, Integer> entry : rgb.entrySet()) {
                telemetry.addData(entry.getKey() + ": ", entry.getValue());
            }
        }

        return rgb;
    }

    /**
     * Overloaded method for getting the RGB values without telemetry.
     * @return Map of the RGB values (Keys: Red, Green, Blue)
     */
    public Map<String, Integer> getRGB() {
        return getRGB(null);
    }

    /**
     * Gets a map of the HSV values detected by the color sensor.
     * @param telemetry Telemetry object to display the values on the driver station (optional)
     * @return Map of the HSV values (Keys: Hue, Saturation, Value)
     */
    public Map<String, Object> getHSV(@Nullable Telemetry telemetry) {
        float[] hsv = new float[3];
        Map<String, Object> hsvMap = new HashMap<>();
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);

        hsvMap.put("Hue", hsv[0]);
        hsvMap.put("Saturation", hsv[1]);
        hsvMap.put("Value", hsv[2]);

        if (telemetry != null) {
            for (Map.Entry<String, Object> entry : hsvMap.entrySet()) {
                telemetry.addData(entry.getKey() + ": ", entry.getValue());
            }
        }

        return hsvMap;
    }

    /**
     * Overloaded method for getting the HSV values without telemetry.
     * @return Map of the HSV values (Keys: Hue, Saturation, Value)
     */
    public Map<String, Object> getHSV() {
        return getHSV(null);
    }
}