package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Mugurel;


@TeleOp(name = "Restaurare Turela", group = "TeleOp")
public class Turelamodif extends LinearOpMode {
    private Mugurel robot;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx driver2 = new GamepadEx(gamepad2);
        robot = new Mugurel(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            driver2.readButtons();

            update_turela(driver2);
            update_glisiera(driver2);

            telemetry.addData("Turela Ticks", robot.turela.getTicks());
            telemetry.addData("Glisiera Ticks", robot.glisiera.getTicks());
            telemetry.update();
        }
        robot.shutdown_system_motors();
    }

    private void update_glisiera(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.X))
            robot.glisiera.modifyPosition(+100);

        if (gamepad.wasJustPressed(GamepadKeys.Button.Y))
            robot.glisiera.modifyPosition(-100);
    }

    private void update_turela(GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.B))
            robot.turela.modifyPosition(+50);

        if (gamepad.wasJustPressed(GamepadKeys.Button.A))
            robot.turela.modifyPosition(-50);

        if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            robot.turela.modifyPosition(-20);

        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            robot.turela.modifyPosition(+20);
    }
}
