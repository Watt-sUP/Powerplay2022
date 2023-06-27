package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ScanPoleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@com.acmerobotics.dashboard.config.Config
@TeleOp(name = "Sensor Test", group = "TeleOp")
public class SensorTest extends CommandOpMode {

    @Override
    public void initialize() {
        DriveSubsystem driveSystem = new DriveSubsystem(hardwareMap,
                "LF", "RF", "LB", "RB");
        SensorSubsystem sensorSystem = new SensorSubsystem(hardwareMap, "distance");
        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.claw, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, 0, 300), 0.2
        );
        GlisiereSubsystem glisiereSystem = new GlisiereSubsystem(
                hardwareMap.dcMotor.get(Config.glisiera),
                hardwareMap.dcMotor.get(Config.glisiera1),
                new SimpleServo(hardwareMap, Config.ghidaj, 0, 300), true
        );
        TurelaSubsystem turelaSystem = new TurelaSubsystem(
                new Motor(hardwareMap, Config.turela),
                () -> glisiereSystem.position != 0
        );

        register(driveSystem);
        register(sensorSystem);
        register(glisiereSystem);
        register(turelaSystem);
        register(colectareSystem);

        GamepadEx driver = new GamepadEx(gamepad1);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver::getLeftY, driver::getLeftX, driver::getRightX);
        driveSystem.setDefaultCommand(driveCommand);

        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> glisiereSystem.setToPosition(4)));
        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> glisiereSystem.setToPosition(0)));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ScanPoleCommand(turelaSystem, sensorSystem, -800, ScanPoleCommand.Direction.LEFT, 50.0), true);
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ScanPoleCommand(turelaSystem, sensorSystem, 800, ScanPoleCommand.Direction.RIGHT, 50.0), true);

        schedule(new RunCommand(() -> {
            telemetry.addData("Current Distance", sensorSystem.getDistance());
            telemetry.addData("Current Ticks", turelaSystem.getTicks());
            telemetry.update();
        }));
    }
}
