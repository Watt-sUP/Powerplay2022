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

    private boolean isWithinThreshold(float value, Pair<Float, Float> threshold) {
        return MathUtils.clamp(value, threshold.first, threshold.second) == value;
    }

    public boolean isColorDetected() {
        float[] currentColor = new float[3];
        colorSensor.RGBtoHSV(colorSensor.red(), colorSensor.blue(), colorSensor.green(), currentColor);

        return isWithinThreshold(currentColor[0], colorThreshold[0]) &&
                isWithinThreshold(currentColor[1], colorThreshold[1]) &&
                isWithinThreshold(currentColor[2], colorThreshold[2]);
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}