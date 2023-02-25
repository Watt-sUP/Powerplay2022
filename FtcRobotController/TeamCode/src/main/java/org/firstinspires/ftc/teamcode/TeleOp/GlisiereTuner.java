package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.commands.subsystems.GlisierePIDSubsystem;
import org.firstinspires.ftc.teamcode.hardware.Config;

@TeleOp
public class GlisiereTuner extends CommandOpMode {
    private int target = 0;

    @Override
    public void initialize() {

        PhotonCore.enable();
        telemetry.setAutoClear(true);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        GamepadEx gamepad = new GamepadEx(gamepad1);

        GlisierePIDSubsystem glisiereSystem = new GlisierePIDSubsystem(
                hardwareMap.get(DcMotorEx.class, Config.glisiera),
                hardwareMap.get(DcMotorEx.class, Config.glisiera1),
                new SimpleServo(hardwareMap, Config.ghidaj, 0, 300)
        );
        register(glisiereSystem);
        new RunCommand(() -> {
            telemetry.addData("Current Target", target);
            telemetry.addData("Current Ticks", glisiereSystem.getTicks());
            telemetry.update();
        }).schedule();

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        () -> {
                            glisiereSystem.setToPosition(3);
                            target = glisiereSystem.positions[3];
                        },
                        () -> {
                            glisiereSystem.setToPosition(1);
                            target = glisiereSystem.positions[1];
                        }
                );
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(() -> {
                    glisiereSystem.setToPosition(0);
                    target = 0;
                }, () -> {
                    glisiereSystem.setToPosition(4);
                    target = glisiereSystem.positions[4];
                });
    }
}
