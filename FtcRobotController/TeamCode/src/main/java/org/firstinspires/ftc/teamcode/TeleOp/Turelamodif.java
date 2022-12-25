package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.Mugurel;
import org.firstinspires.ftc.teamcode.gamepad.Button;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;


@TeleOp(name = "Restaurare Turela", group = "TeleOp")
public class Turelamodif extends LinearOpMode {
    private Mugurel robot;
    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx l = new GamepadEx(gamepad1);
        GamepadEx b = new GamepadEx(gamepad2);
        robot = new Mugurel(hardwareMap);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            l.update();
            b.update();

            update_turela(b.a, b.b);
            update_glisiera(b.x, b.y);
            telemetry.addData("Turela ticks:", robot.turela.getTicks());
            telemetry.addData("Glisiera ticks:", robot.glisiera.getTicks());
            telemetry.update();
            idle();

        }
        robot.shutdown_system_motors();
    }

    private void update_glisiera(Button pos_down, Button pos_up) {
        if (pos_down.pressed()) {
            robot.glisiera.modifyPosition(+ 100);
        }
        if (pos_up.pressed()) {
            robot.glisiera.modifyPosition( -100);
        }
    }

    private void update_turela(Button pos_up, Button pos_down) {
        if (pos_down.pressed())
            robot.turela.modifyPosition(+ 50);

        if (pos_up.pressed())
            robot.turela.modifyPosition( -50);
    }
}
