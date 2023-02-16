package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@TeleOp(name = "Drive Optimizat (Tomoiu + Vulpoiu)", group = "TeleOp")
public class ControlatTomoiu extends CommandOpMode {

    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void initialize() {

        Trigger limiter = new Trigger(() -> gamepad1.right_trigger > 0.3);
        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, Config.left_front),
                new Motor(hardwareMap, Config.right_front),
                new Motor(hardwareMap, Config.left_back),
                new Motor(hardwareMap, Config.right_back)
        );
        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.claw, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360), 0.25
        );
        GlisiereSubsystem glisiereSystem = new GlisiereSubsystem(
                hardwareMap.dcMotor.get(Config.glisiera),
                hardwareMap.dcMotor.get(Config.glisiera1)
        );
        TurelaSubsystem turelaSystem = new TurelaSubsystem(
                new Motor(hardwareMap, Config.turela),
                () -> glisiereSystem.position != 0
        );
        time.reset();

        register(driveSystem);
        register(glisiereSystem);
        register(turelaSystem);
        register(colectareSystem);
        schedule(new RunCommand(() -> {
            telemetry.addLine((int) time.seconds() + " seconds elapsed OpMode start");
            telemetry.update();
        }));

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        driveSystem.setDefaultCommand(driveCommand);
        // Drivetrain controls below
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> driveSystem.setPowerLimit(0.2))
                .whenReleased(() -> driveSystem.setPowerLimit(1.0));
        limiter.whileActiveContinuous(() -> driveSystem.setPowerLimit(0.4))
                .whenInactive(() -> driveSystem.setPowerLimit(1.0));

        // Turret controls below
        driver1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.LEFT));
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.RIGHT));
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.FORWARD));
        driver1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.BACKWARDS));

        // Slider controls below
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> glisiereSystem.setToPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> glisiereSystem.setToPosition(0));
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> glisiereSystem.setToPosition(glisiereSystem.position + 1));
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> glisiereSystem.setToPosition(glisiereSystem.position - 1));
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> glisiereSystem.modifyTicks(-160));

        // Collector controls below
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(colectareSystem::toggleClaw);
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(colectareSystem::toggleScissors);
    }
}