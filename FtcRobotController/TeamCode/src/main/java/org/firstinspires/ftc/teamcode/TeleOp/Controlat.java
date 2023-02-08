package org.firstinspires.ftc.teamcode.TeleOp;


import androidx.annotation.NonNull;

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
import java.util.function.Supplier;

@TeleOp(name = "Salam adevaratu", group = "TeleOp")
public class Controlat extends LinearOpMode {

    private Mugurel robot;
    private int pos_sliders = 0, last_pos_sliders = 0;
    private final Supplier<Boolean> turretSafe = () -> pos_sliders != 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mugurel(hardwareMap);
        robot.turela.enableSafety(turretSafe);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        DecimalFormat df = new DecimalFormat("0.00");

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        robot.driveMotors.reverse_motors("Right");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(200);
        waitForStart();

        while (opModeIsActive()) {
            timer.reset();

            driver1.readButtons();
            driver2.readButtons();

            int offset = robot.glisiera.getOffset();

            double powerLimit;
            if (gamepad1.right_trigger >= 0.3)
                powerLimit = 0.2;
            else if (driver1.isDown(GamepadKeys.Button.RIGHT_BUMPER))
                powerLimit = 0.4;
            else
                powerLimit = 1.0;

            robot.driveMotors.update_motor_speed(gamepad1, powerLimit);
            check_claw(driver2);
            check_sliders(driver2);
            check_turret(driver1);

            if (driver2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))
                robot.foarfeca.toggleFoarfeca();

            telemetry.addData("Current Sliders Offset", offset);
            telemetry.addData("Current Turela Ticks", robot.turela.getTicks());
            telemetry.addData("Power Limit", powerLimit);
            telemetry.addLine("OpMode is running at " + df.format(1000 / timer.milliseconds()) + " Hz");
            telemetry.update();

            idle();
        }
        robot.shutdown_system_motors();
    }

    /**
     * <p>Helper method to handle claw movement based on gamepad input.</p>
     * @param gamepad GamepadEx object, used to read gamepad input and toggle the claw.
     */
    private void check_claw(@NonNull GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.A))
            robot.claw.toggleClaw();
    }

    /**
     * <p>Helper method to handle turret movement based on gamepad input.</p>
     * @param gamepad GamepadEx object, used to read gamepad input and execute actions.
     */
    private void check_turret(@NonNull GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.X))
            robot.turela.setToPosition(Direction.LEFT);

        if (gamepad.wasJustPressed(GamepadKeys.Button.B))
            robot.turela.setToPosition(Direction.RIGHT);

        if (gamepad.wasJustPressed(GamepadKeys.Button.Y))
            robot.turela.setToPosition(Direction.FORWARD);

        if (gamepad.wasJustPressed(GamepadKeys.Button.A))
            robot.turela.setToPosition(Direction.BACKWARDS);
    }

    /**
     * <p>Helper method to handle slider movement based on gamepad input.</p>
     * @param gamepad GamepadEx object, used to read gamepad input and execute actions.
     */
    private void check_sliders(@NonNull GamepadEx gamepad) {
        if (gamepad.wasJustPressed(GamepadKeys.Button.Y))
            pos_sliders++;

        if (gamepad.wasJustPressed(GamepadKeys.Button.X))
            pos_sliders--;

        if (gamepad.wasJustPressed(GamepadKeys.Button.B))
            robot.glisiera.modifyPosition(-160);

        if (gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
            pos_sliders = 4;

        if (gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            pos_sliders = 0;

        pos_sliders = Range.clip(pos_sliders, 0, 4);
        if (pos_sliders != last_pos_sliders) {
            robot.glisiera.setToPosition(pos_sliders);
            last_pos_sliders = pos_sliders;
        }
    }
}