package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.gamepad.Button;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;

@TeleOp(name = "Salam adevaratu", group = "Testing")
public class lll extends LinearOpMode {

    //Declaratii
    private boolean faceChanged = false, faceIsHeld = false;
    private boolean deget = true;
    private Mugurel robot;
    private int pos_glisiera = 0;
    private int pos_turela = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mugurel(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Config.left_front);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Config.left_back);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Config.right_front);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Config.right_back);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {

            double acceleration = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * -1.1;
            double rotation = gamepad1.right_stick_x;

            double powerLimit = 1.0;
            if (gamepad1.left_trigger >= 0.3)
                powerLimit = 0.4;
            else if (gamepad1.right_trigger >= 0.3)
                powerLimit = 0.3;
            else
                powerLimit = 1.0;

            if (gamepad1.y && !faceIsHeld) {
                faceIsHeld = true;
                faceChanged = !faceChanged;
            } else if (!gamepad1.y) faceIsHeld = false;

            if (!faceChanged) {
                frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

                rotation = rotation * (-1);
            } else {
                frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            double denominator = Math.max(Math.abs(acceleration) + Math.abs(strafe) + Math.abs(rotation), 1);
            double frontLeftPower = (acceleration + strafe + rotation) / denominator;
            double backLeftPower = (acceleration - strafe + rotation) / denominator;
            double frontRightPower = (acceleration - strafe - rotation) / denominator;
            double backRightPower = (acceleration + strafe - rotation) / denominator;

            frontLeftMotor.setPower(Range.clip(frontLeftPower, -powerLimit, powerLimit));
            backLeftMotor.setPower(Range.clip(backLeftPower, -powerLimit, powerLimit));
            frontRightMotor.setPower(Range.clip(frontRightPower, -powerLimit, powerLimit));
            backRightMotor.setPower(Range.clip(backRightPower, -powerLimit, powerLimit));


            // robot.deget(gamepad2.pula_mea);
            robot.glisiera(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.b, gamepad2.right_bumper, gamepad2.left_bumper);
            robot.turela(gamepad2.dpad_right, gamepad2.dpad_left, gamepad2.a, gamepad2.x, gamepad2.y);

            idle();
        }
        robot.glisiera.motor.setPower(0);
        robot.turela.motor.setPower(0);
    }

   private void deget(Button deget) {
       if (deget.pressed())
           robot.deget.toggleDeget();
    }

    private void turela(Button pos_up, Button pos_down, Button b, Button addB, Button subB) {
        if(pos_up.pressed()) {
            pos_turela++;
            if(pos_turela > 4)
                pos_turela = 4;
            robot.turela.setToPosition(pos_turela);
        }

        if(pos_down.pressed()) {
            pos_turela--;
            if(pos_turela < 0)
                pos_turela = 0;
            robot.turela.setToPosition(pos_turela);
        }

        if(addB.pressed())
            robot.turela.modifyPosition(200);
        if(subB.pressed())
            robot.turela.modifyPosition(-200);
    }

    private void glisiera(Button pos_up, Button pos_down, Button b, Button addB, Button subB) {
        if(pos_up.pressed()) {
            pos_glisiera++;
            if(pos_glisiera > 4)
                pos_glisiera = 4;
            robot.glisiera.setToPosition(pos_glisiera);
        }

        if(pos_down.pressed()) {
            pos_glisiera--;
            if(pos_glisiera < 0)
                pos_glisiera = 0;
            robot.glisiera.setToPosition(pos_glisiera);
        }

        if(addB.pressed())
            robot.glisiera.modifyPosition(200);
        if(subB.pressed())
            robot.glisiera.modifyPosition(-200);
    }

    private void deget(Button cupa) {
        if (cupa.pressed())
            robot.deget.toggleCupa();
    }
}