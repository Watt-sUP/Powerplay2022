package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@TeleOp(name = "Drive Optimizat")
public class ControlatOptimizat extends CommandOpMode {

    @Override
    public void initialize() {

        GamepadEx gamepad = new GamepadEx(gamepad1);
        GamepadTrigger right_trig = new GamepadTrigger(gamepad, GamepadKeys.Trigger.RIGHT_TRIGGER);

        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, Config.left_front),
                new Motor(hardwareMap, Config.right_front),
                new Motor(hardwareMap, Config.left_back),
                new Motor(hardwareMap, Config.right_back),
                gamepad
        );

        register(driveSystem);
        driveSystem.setDefaultCommand(new RunCommand(driveSystem::drive, driveSystem));
        schedule(new RunCommand(() -> driveSystem.getPowerLimit(telemetry)));

        while (opModeIsActive()) {
            gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                    .toggleWhenActive(() -> driveSystem.setPowerLimit(0.4), () -> driveSystem.setPowerLimit(1.0));
            right_trig.toggleWhenActive(() -> driveSystem.setPowerLimit(0.2), () -> driveSystem.setPowerLimit(1.0));

            telemetry.update();
        }

        reset();
    }

    static class GamepadTrigger extends Trigger {
        GamepadEx gamepad;
        GamepadKeys.Trigger trigger;
        TriggerReader reader;

        public GamepadTrigger(GamepadEx gamepad, GamepadKeys.Trigger trigger) {
            this.gamepad = gamepad;
            this.trigger = trigger;
            reader = new TriggerReader(this.gamepad, this.trigger);
        }

        @Override
        public boolean get() {
            return reader.isDown();
        }
    }
}