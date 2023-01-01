package org.firstinspires.ftc.teamcode.TeleOp;


import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.gamepad.Button;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;

@TeleOp(name = "Salam adevaratu", group = "TeleOp")
public class Controlat extends LinearOpMode {

    private Mugurel robot;
    private int max_offset;
    private int pos_glisiera = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Mugurel(hardwareMap);
        GamepadEx l = new GamepadEx(gamepad1);
        GamepadEx b = new GamepadEx(gamepad2);
        Servo odoServo = hardwareMap.get(Servo.class, Config.odometry_servo);

        robot.driveMotors.reverse_motors("right");
        odoServo.setPosition(Config.odo_pos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Odometry servo lifted to position" + Config.odo_pos);
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {
            l.update();
            b.update();

            int offset = robot.glisiera.getOffset();
            if (offset > max_offset)
                max_offset = offset;

            @Nullable Double powerLimit;
            if (gamepad1.right_trigger >= 0.3)
                powerLimit = 0.2;
            else if (gamepad1.right_bumper)
                powerLimit = 0.4;
            else
                powerLimit = null;

            deget(b.a);
            robot.driveMotors.update_motor_speed(gamepad1, powerLimit, null);
            glisiera(b.y, b.x, b.b, b.right_bumper, b.left_bumper);
            turela(b.dpad_up, b.dpad_down, b.dpad_right, b.dpad_left);

            telemetry.addData("Current Offset", offset);
            telemetry.addData("Max Offset", max_offset);
            telemetry.update();

            idle();
        }
        robot.shutdown_system_motors();
    }

    private void deget(Button y) {
        if (y.pressed())
            robot.deget.toggleDeget();
    }

    private void turela(Button pos_up, Button pos_down, Button addA, Button subA) {
        if (addA.pressed())
            robot.turela.setToPosition(1);

        if (subA.pressed())
            robot.turela.setToPosition(3);

        if (pos_down.pressed())
            robot.turela.setToPosition(2);

        if (pos_up.pressed())
            robot.turela.setToPosition(4);
    }

    private void glisiera(Button pos_up, Button pos_down, Button b, Button addB, Button subB) {
        if (pos_up.pressed() && pos_glisiera < 5) {
            pos_glisiera++;
            robot.glisiera.setToPosition(pos_glisiera);
        }

        if (b.pressed())
            robot.glisiera.modifyPosition(-100);

        if (pos_down.pressed() && pos_glisiera > 0) {
            pos_glisiera--;
            robot.glisiera.setToPosition(pos_glisiera);
        }

        if (addB.pressed()) {
            pos_glisiera = 5;
            robot.glisiera.setToPosition(pos_glisiera);
        }
        if (subB.pressed()) {
            pos_glisiera = 0;
            robot.glisiera.setToPosition(pos_glisiera);
        }
    }
}