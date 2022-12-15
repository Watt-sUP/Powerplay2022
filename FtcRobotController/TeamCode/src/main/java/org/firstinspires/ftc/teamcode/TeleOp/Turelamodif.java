package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.Mugurel;
import org.firstinspires.ftc.teamcode.gamepad.Button;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;


@TeleOp(name = "Turelamodif", group = "Testing")
public class Turelamodif extends LinearOpMode {
    private Mugurel robot;
    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx l = new GamepadEx(gamepad1);
        GamepadEx b = new GamepadEx(gamepad2);
        robot = new Mugurel(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            l.update();
            b.update();

            turela(b.a, b.b, l.dpad_right, b.dpad_right, b.dpad_left);
            glisiera(b.x, b.y, l.dpad_right, b.dpad_right);
            telemetry.addData("ticks:",  robot.turela.getTicks() );
            telemetry.addData("ticks:",  robot.glisiera.getTicks() );
            telemetry.update();
            idle();

        }
        robot.turela.motortur.setPower(0);
        robot.glisiera.motor.setPower(0);
    }
    private void glisiera(Button pos_down, Button pos_up, Button a, Button b) {
        if (pos_down.pressed()) {
            robot.glisiera.modifyPosition(+ 100);
        } ;
        if (pos_up.pressed()) {
            robot.glisiera.modifyPosition( -100);
        } ;
    }
    private void turela(Button pos_up, Button pos_down, Button b, Button addA, Button subA) {
//pos_up=dpad_up pos_down=dpad_down addA=dpad_right subA=dpad_left
        if (pos_down.pressed()) {
            robot.turela.modifyPosition(+ 50);
        } ;
        if (pos_up.pressed()) {
            robot.turela.modifyPosition( -50);
        } ;
    }



}
