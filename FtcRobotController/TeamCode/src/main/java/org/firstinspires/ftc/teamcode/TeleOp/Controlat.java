package org.firstinspires.ftc.teamcode.TeleOp;


import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;
import org.firstinspires.ftc.teamcode.hardware.Turela;

@TeleOp(name = "Salam adevaratu", group = "TeleOp")
public class Controlat extends LinearOpMode {

    private Mugurel robot;
    private int max_offset;
    private int pos_glisiera = 0, last_pos_glisiera = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mugurel(hardwareMap);
        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        Servo odoServo = hardwareMap.get(Servo.class, Config.odometry_servo);

        robot.driveMotors.reverse_motors("Right");
        odoServo.setPosition(Config.odo_pos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Odometry servo lifted to position " + Config.odo_pos);
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {
            driver1.readButtons();
            driver2.readButtons();

            int offset = robot.glisiera.getOffset();
            if (offset > max_offset)
                max_offset = offset;

            @Nullable Double powerLimit;
            if (gamepad1.right_trigger >= 0.3)
                powerLimit = 0.2;
            else if (driver1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
                powerLimit = 0.4;
            else
                powerLimit = null;

            deget(driver1);
            robot.driveMotors.update_motor_speed(gamepad1, powerLimit, null);
            glisiera(driver2);
            turela(driver2);

            if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))
                robot.foarfeca.toggleFoarfeca();

            telemetry.addData("Current Offset", offset);
            telemetry.addData("Max Offset", max_offset);
            telemetry.addData("Current Sliders Position", pos_glisiera);
            telemetry.addData("Power Limit", (powerLimit == null) ? 1.0 : powerLimit);
            telemetry.update();

            idle();
        }
        robot.shutdown_system_motors();
    }

    private void deget(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.A))
            robot.deget.toggleDeget();
    }

    private void turela(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            robot.turela.setToPosition(Turela.Position.LEFT);

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            robot.turela.setToPosition(Turela.Position.BACK);

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            robot.turela.setToPosition(Turela.Position.RIGHT);

        if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            robot.turela.setToPosition(Turela.Position.FRONT);
    }

    private void glisiera(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y))
            pos_glisiera++;

        if (gamepad.wasJustPressed(GamepadKeys.Button.X))
            pos_glisiera--;

        if (gamepad.wasJustPressed(GamepadKeys.Button.B))
            robot.glisiera.modifyPosition(-80);

        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            pos_glisiera = 4;

        if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            pos_glisiera = 0;

        pos_glisiera = Range.clip(pos_glisiera, 0, 4);
        if (pos_glisiera != last_pos_glisiera) {
            robot.glisiera.setToPosition(pos_glisiera);
            last_pos_glisiera = pos_glisiera;
        }
    }
}