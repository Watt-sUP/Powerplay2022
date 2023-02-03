package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@TeleOp(name = "Drive Optimizat")
public class ControlatOptimizat extends CommandOpMode {

    @Override
    public void initialize() {

        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, Config.left_front),
                new Motor(hardwareMap, Config.right_front),
                new Motor(hardwareMap, Config.left_back),
                new Motor(hardwareMap, Config.right_back)
        );

        GamepadEx gamepad = new GamepadEx(gamepad1);
        DriveCommand driveCommand = new DriveCommand(driveSystem, gamepad::getLeftY, gamepad::getLeftX, gamepad::getRightX);
//        ColectareSubsystem colectareSystem = new ColectareSubsystem(
//                new SimpleServo(hardwareMap, Config.deget, -360, 360),
//                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360)
//        );

        register(driveSystem);
        driveSystem.setDefaultCommand(driveCommand);
//        register(colectareSystem);
//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .toggleWhenActive(() -> driveSystem.setPowerLimit(0.2), () -> driveSystem.setPowerLimit(1.0));
    }
}