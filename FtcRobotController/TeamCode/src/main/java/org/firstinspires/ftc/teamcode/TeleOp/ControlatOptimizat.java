package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@TeleOp(name = "Drive Optimizat")
public class ControlatOptimizat extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx gamepad = new GamepadEx(gamepad1);

        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, Config.left_front),
                new Motor(hardwareMap, Config.right_front),
                new Motor(hardwareMap, Config.left_back),
                new Motor(hardwareMap, Config.right_back),
                gamepad
        );
        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.deget, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360)
        );

        register(driveSystem);
        register(colectareSystem);

        driveSystem.setDefaultCommand(new RunCommand(driveSystem::drive, driveSystem));
        schedule(new RunCommand(() -> driveSystem.getPowerLimit(telemetry)));

        while (opModeIsActive()) {
            gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .toggleWhenActive(() -> driveSystem.setPowerLimit(0.2), () -> driveSystem.setPowerLimit(1.0));

            gamepad.getGamepadButton(GamepadKeys.Button.X)
                    .whenPressed(new InstantCommand(colectareSystem::toggleGheara), false);
            gamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whenPressed(new InstantCommand(colectareSystem::toggleFoarfeca), false);
            telemetry.update();
        }

        reset();
    }
}