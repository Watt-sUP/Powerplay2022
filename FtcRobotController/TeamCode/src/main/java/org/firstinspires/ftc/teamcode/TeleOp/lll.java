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
public class DriverControlledGaju_BLUE extends LinearOpMode {

    //Declaratii
    private boolean faceChanged = false, faceIsHeld = false;
    private boolean deget =true;
    private Mugurel robot;
    private int pos_glisiere = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mugurel(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Config.right_back);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Config.right_front);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Config.left_back);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Config.left_front);


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while (opModeIsActive()) {



            double acceleration = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * -1.1;
            double rotation = gamepad1.right_stick_x;


            double powerLimit = 1.0;
            if (gamepad1.left_trigger >= 0.3) {
                powerLimit = 0.4;
            } else if (gamepad1.right_trigger >= 0.3) {
                powerLimit = 0.3;
            } else {
                powerLimit = 1.0;
            }

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


            deget(gamepad2.b);
            glisiere(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.b, gamepad2.right_bumper, gamepad2.left_bumper);


            idle();

        }
        robot.glisiere.motor.setPower(0);
    }

   private void deget(Button deget) {
       if (deget.pressed())
           robot.deget.toggleDeget();}

    private void glisiere(Button pos_up, Button pos_down, Button b, Button addB, Button subB) {

        if(pos_up.pressed()) {
            pos_glisiere++;
            if(pos_glisiere > 4)    pos_glisiere = 4;
            robot.glisiere.setToPosition(pos_glisiere);
        }

        if(pos_down.pressed()) {
            pos_glisiere--;
            if(pos_glisiere < 0)    pos_glisiere = 0;
            robot.glisiere.setToPosition(pos_glisiere);
        }


        if(addB.pressed())
            robot.glisiere.modifyPosition(200);
        if(subB.pressed())
            robot.glisiere.modifyPosition(-200);
    }

    private void deget(Button cupa) {
        if (cupa.pressed()) {
            
            robot.deget.toggleCupa();

        }




    }


}