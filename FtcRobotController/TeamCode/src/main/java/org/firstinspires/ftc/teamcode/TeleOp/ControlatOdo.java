package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Direction;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@TeleOp(name = "Drive Optimizat (Odo + Leica)", group = "TeleOp")
public class ControlatOdo extends CommandOpMode {
    private final ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void initialize() {

        PhotonCore.enable();
        ServoEx ghidaj = new SimpleServo(hardwareMap, Config.ghidaj, 0, 300);
        ghidaj.turnToAngle(150);

        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, Config.left_front),
                new Motor(hardwareMap, Config.right_front),
                new Motor(hardwareMap, Config.left_back),
                new Motor(hardwareMap, Config.right_back)
        );
        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.claw, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360), 0.225
        );
        GlisiereSubsystem glisiereSystem = new GlisiereSubsystem(
                hardwareMap.get(DcMotorEx.class, Config.glisiera),
                hardwareMap.get(DcMotorEx.class, Config.glisiera1),
                new SimpleServo(hardwareMap, Config.ghidaj, 0, 300)
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
            telemetry.addData("Current Power Limit", driveSystem.getPowerLimit());
            telemetry.update();
        }));

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        DriveCommand driveCommand = new DriveCommand(driveSystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        driveSystem.setDefaultCommand(driveCommand);
        // Drivetrain controls below
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> driveSystem.setPowerLimit(0.2))
                .whenReleased(() -> driveSystem.setPowerLimit(driveSystem.defaultPowerLimit));
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> driveSystem.setPowerLimit(0.4))
                .whenReleased(() -> driveSystem.setPowerLimit(driveSystem.defaultPowerLimit));
        driver1.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(() -> {
                    driveSystem.defaultPowerLimit = 0.8;
                    driveSystem.setPowerLimit(driveSystem.defaultPowerLimit);
                }, () -> {
                    driveSystem.defaultPowerLimit = 1.0;
                    driveSystem.setPowerLimit(driveSystem.defaultPowerLimit);
                });

        // Turret controls below
        driver1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.LEFT));
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.RIGHT));
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> turelaSystem.setToPosition(Direction.FORWARD));

        // Slider controls below
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ParallelCommandGroup(
                        // Raises the sliders and rotates the turret to the front
                        new InstantCommand(() -> glisiereSystem.setToPosition(4)),
                        new WaitUntilCommand(() -> glisiereSystem.position != 0)
                                .andThen(new InstantCommand(() -> turelaSystem.setToPosition(Direction.FORWARD)))
                ));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        // Lowers the sliders and rotates the turret to the last known position
                        // or the front if it is unknown
                        new InstantCommand(() -> turelaSystem.setToPosition(turelaSystem.lastDirection)),
                        new InstantCommand(() -> glisiereSystem.setToPosition(0))
                ));

        // Move the slides to the highest and lowest positions
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> glisiereSystem.setToPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> glisiereSystem.setToPosition(0));

        // Raise and lower the sliders by 1 position
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> glisiereSystem.setToPosition(glisiereSystem.position + 1));
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> glisiereSystem.setToPosition(glisiereSystem.position - 1));

        // Lowers the slides a bit
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> glisiereSystem.modifyTicks(-160));

        // Collector controls below
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> {
                    glisiereSystem.lowerUnghi();
                    colectareSystem.toggleClaw();
                });
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(colectareSystem::toggleScissors);
    }
}