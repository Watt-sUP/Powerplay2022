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

    public static int HUE_MIN = 10 * 2, HUE_MAX = 60 * 2;
    public static float SATURATION_MIN = (float) (150.0 / 255.0), SATURATION_MAX = 1;
    public static float VALUE_MIN = (float) (115.0 / 255.0), VALUE_MAX = 1;

    @Override
    public void initialize() {
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap,
                "left_front", "right_front", "left_back", "right_back");
        SensorSubsystem sensorSystem = new SensorSubsystem(hardwareMap, "color");


        register(driveSystem);
        register(sensorSystem);

        GamepadEx driver = new GamepadEx(gamepad1);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver::getLeftY, driver::getLeftX, driver::getRightX);
        driveSystem.setDefaultCommand(driveCommand);

        schedule(new InstantCommand(() ->
                sensorSystem.setColorThreshold(
                        new Pair<>((float) HUE_MIN, (float) HUE_MAX),
                        new Pair<>(SATURATION_MIN, SATURATION_MAX),
                        new Pair<>(VALUE_MIN, VALUE_MAX)
                ))
        );
        schedule(new RunCommand(() -> {
            Pair<Float, Float>[] colorThreshold = sensorSystem.getColorThreshold();

            telemetry.addData("Current Distance", sensorSystem.getDistance());
            telemetry.addLine("Current Color Threshold:" +
                            "\nHue: " + colorThreshold[0].first + "-" + colorThreshold[0].second +
                            "\nSaturation: " + colorThreshold[1].first + "-" + colorThreshold[1].second +
                            "\nValue: " + colorThreshold[2].first + "-" + colorThreshold[2].second);
            telemetry.addData("Current RGB Values", sensorSystem.getRGB());
            telemetry.addData("Current HSV Values", sensorSystem.getHSV());
            telemetry.addData("Color Found", sensorSystem.isColorDetected() ? "Yes" : "No");
            telemetry.update();
        }));
    }
}
