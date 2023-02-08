package org.firstinspires.ftc.teamcode.TeleOp;


import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Mugurel;

import java.text.DecimalFormat;

@TeleOp(name = "Salam adevaratu", group = "TeleOp")
public class Controlat extends LinearOpMode {

    private Mugurel robot;
    private int max_offset;
    private int pos_glisiera = 0, last_pos_glisiera = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mugurel(hardwareMap);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        DecimalFormat df = new DecimalFormat("0.00");
//        robot.turela.motortur.resetEncoder();

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        robot.driveMotors.reverse_motors("Right");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {
            timer.reset();

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

            deget(driver2);
            robot.driveMotors.update_motor_speed(gamepad1, powerLimit);
            glisiera(driver2);
            turela(driver1);

            if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))
                robot.foarfeca.toggleFoarfeca();

            telemetry.addData("Current Sliders Offset", offset);
            telemetry.addData("Current Turela Ticks", robot.turela.getTicks());
            telemetry.addData("Power Limit", (powerLimit == null) ? 1.0 : powerLimit);
            telemetry.addLine("OpMode is running at " + df.format(1000 / timer.milliseconds()) + " Hz");
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
        robot.turela.glis_position = pos_glisiera;
        if (gamepad.wasJustPressed(GamepadKeys.Button.X))
            robot.turela.setToPosition(Direction.LEFT);

        if (gamepad.wasJustPressed(GamepadKeys.Button.B))
            robot.turela.setToPosition(Direction.RIGHT);

        if (gamepad.wasJustPressed(GamepadKeys.Button.Y))
            robot.turela.setToPosition(Direction.FORWARD);

        if (gamepad.wasJustPressed(GamepadKeys.Button.A))
            robot.turela.setToPosition(Direction.BACKWARDS);

    }

    private void glisiera(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y))
            pos_glisiera++;

        if (gamepad.wasJustPressed(GamepadKeys.Button.X))
            pos_glisiera--;

        if (gamepad.wasJustPressed(GamepadKeys.Button.B))
            robot.glisiera.modifyPosition(-160);

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