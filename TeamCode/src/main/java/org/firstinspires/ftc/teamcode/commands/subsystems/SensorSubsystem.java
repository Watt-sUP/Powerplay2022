package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Command-based subsystem for the color and proximity recognition sensors.
 */
public class SensorSubsystem extends SubsystemBase {
    private final SensorColor colorSensor;
    private final SensorRevTOFDistance distanceSensor;
    private Pair<Float, Float>[] colorThreshold = new Pair[]{
            new Pair<>(0f, 360f),
            new Pair<>(0f, 1f),
            new Pair<>(0f, 1f)
    };

    /**
     * Creates a new instance of the subsystem.
     *
     * @param hardwareMap HardwareMap object for importing the sensors
     * @param sensor      String representing the name of the sensor
     */
    public SensorSubsystem(HardwareMap hardwareMap, String sensor) {
        colorSensor = new SensorColor(hardwareMap, sensor);
        distanceSensor = new SensorRevTOFDistance(hardwareMap, sensor);
    }

    public void setColorThreshold(Pair<Float, Float> hue, Pair<Float, Float> saturation, Pair<Float, Float> value) {
        colorThreshold[0] = hue;
        colorThreshold[1] = saturation;
        colorThreshold[2] = value;
    }

    public Pair<Float, Float>[] getColorThreshold() {
        return colorThreshold;
    }

    /**
     * Checks if a value is within a threshold.
     * @param value The value to evaluate
     * @param threshold The threshold to check against
     * @return Boolean representing if the value is within the threshold
     */
    private boolean isWithinThreshold(float value, Pair<Float, Float> threshold) {
        return MathUtils.clamp(value, threshold.first, threshold.second) == value;
    }

    /**
     * Checks if the color detected by the sensor fits the current threshold.
     * @return Boolean that evaluates to true if the color is within the threshold
     */
    public boolean isColorDetected() {
        float[] currentColor = new float[3];
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.blue(), colorSensor.green(), currentColor);

        return isWithinThreshold(currentColor[0], colorThreshold[0]) &&
                isWithinThreshold(currentColor[1], colorThreshold[1]) &&
                isWithinThreshold(currentColor[2], colorThreshold[2]);
    }

    /**
     * Gets the RGB values of the color detected by the sensor.
     * @return Array of floats representing the RGB values
     */
    public float[] getRGB() {
        return new float[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
    }

    /**
     * Gets the HSV values of the color detected by the sensor.
     * @return Array of floats representing the HSV values
     */
    public float[] getHSV() {
        float[] currentColor = new float[3];
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.blue(), colorSensor.green(), currentColor);
        return currentColor;
    }

    /**
     * Gets the distance detected by the proximity sensor.
     * @return Double representing the distance in centimeters
     */
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}