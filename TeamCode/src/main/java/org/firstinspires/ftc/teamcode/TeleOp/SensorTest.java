package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;

@Config
@TeleOp(name = "Sensor Test", group = "TeleOp")
public class SensorTest extends CommandOpMode {

    public static int HUE_MIN = 320, HUE_MAX = 350;
    public static double SATURATION_MIN = 0.45, SATURATION_MAX = 1;
    public static double VALUE_MIN = 0.1, VALUE_MAX = 1;

    @Override
    public void initialize() {
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap,
                "LF", "RF", "LB", "RB");
        SensorSubsystem sensorSystem = new SensorSubsystem(hardwareMap, "color");


        register(driveSystem);
        register(sensorSystem);

        GamepadEx driver = new GamepadEx(gamepad1);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver::getLeftY, driver::getLeftX, driver::getRightX);
        driveSystem.setDefaultCommand(driveCommand);

        schedule(new InstantCommand(() ->
                sensorSystem.setColorThreshold(
                        new Pair<>((float) HUE_MIN, (float) HUE_MAX),
                        new Pair<>((float) SATURATION_MIN, (float) SATURATION_MAX),
                        new Pair<>((float) VALUE_MIN, (float) VALUE_MAX)
                ))
        );
        schedule(new RunCommand(() -> {
            Pair<Float, Float>[] colorThreshold = sensorSystem.getColorThreshold();
            float[] rgb = sensorSystem.getRGB();
            float[] hsv = sensorSystem.getHSV();

            telemetry.addData("Current Distance", sensorSystem.getDistance());
            telemetry.addLine("Current Color Threshold:" +
                            "\n    Hue: " + colorThreshold[0].first + "-" + colorThreshold[0].second +
                            "\n    Saturation: " + colorThreshold[1].first + "-" + colorThreshold[1].second +
                            "\n    Value: " + colorThreshold[2].first + "-" + colorThreshold[2].second);
            telemetry.addLine("Current RGB Values: " +
                            "\n    Red: " + rgb[0] +
                            "\n    Green: " + rgb[1] +
                            "\n    Blue: " + rgb[2]);
            telemetry.addData("Current HSV Values", "    Hue: " + hsv[0] + "\n    Saturation: " + hsv[1] + "\n    Value: " + hsv[2]);
            telemetry.addData("Color Found", sensorSystem.isColorDetected() ? "Yes" : "No");
            telemetry.update();
        }));
    }
}
