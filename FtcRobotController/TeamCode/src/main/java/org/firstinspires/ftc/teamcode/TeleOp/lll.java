package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.gamepad.Button;
import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;

@TeleOp(name = "Salam adevaratu", group = "Testing")
public class lll extends LinearOpMode {

    //Declaratii
    private boolean faceChanged = false, faceIsHeld = false;
    private boolean deget = true;
    private Mugurel robot;
    private int offset, max_offset;
    private int pos_glisiera = 0;
    private int pos_turela = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mugurel(hardwareMap);
        GamepadEx l = new GamepadEx(gamepad1);
        GamepadEx b = new GamepadEx(gamepad2);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Config.left_front);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Config.left_back);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Config.right_front);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Config.right_back);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            l.update();
            b.update();
            offset = robot.glisiera.getOffset();
            if (max_offset < offset)
                max_offset = offset;

            double acceleration = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x * 1.1;
            double rotation = gamepad1.right_stick_x * -1;

            double powerLimit = 1.0;
            if (gamepad1.right_trigger >= 0.3)
                powerLimit = 0.2;
            else if (gamepad1.right_bumper)
                powerLimit = 0.4;
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

            deget(b.a);
            glisiera(b.y, b.x, b.b, b.right_bumper, b.left_bumper);
            turela(b.dpad_up, b.dpad_down, l.dpad_right, b.dpad_right, b.dpad_left);

            telemetry.addData("Ticks 1:",  robot.glisiera.getTicks());
            telemetry.addData("Ticks 2:",  robot.glisiera.getTickss());
            telemetry.addData("Current Offset:", offset);
            telemetry.addData("Max Offset:", max_offset);
            telemetry.update();

            idle();
        }
        robot.glisiera.motor.setPower(0);
        robot.turela.motortur.setPower(0);
    }

    private void deget(Button y) {
        if (y.pressed())
            robot.deget.toggleDeget();
    }

    private void turela(Button pos_up, Button pos_down, Button b, Button addA, Button subA) {
        //if (pos_up.pressed()) {
        // pos_turela = pos_turela + 1;
        //  if (pos_turela > 4) pos_turela = 4;
        //    robot.turela.setToPosition(pos_turela);
        //  }

        //  if(pos_down.pressed()) {
        //    pos_turela=pos_turela - 1;
        //  if(pos_turela < 0) pos_turela = 0;
        //robot.turela.setToPosition(pos_turela);
        //}
        //pos_up=dpad_up pos_down=dpad_down addA=dpad_right subA=dpad_left
        if (addA.pressed()) {
            pos_turela = 1;
            robot.turela.setToPosition(pos_turela);
        }
        if (subA.pressed()) {
            pos_turela = 3;
            robot.turela.setToPosition(pos_turela);
        }
        if (pos_down.pressed()) {
            pos_turela = 2;
            robot.turela.setToPosition(pos_turela);
        }
        if (pos_up.pressed()) {
            pos_turela = 4;
            robot.turela.setToPosition(pos_turela);
        }
    }

    private void glisiera(Button pos_up, Button pos_down, Button b, Button addB, Button subB) {
        if (pos_up.pressed()) {
            pos_glisiera++;
            if (pos_glisiera > 5)
                pos_glisiera = 5;
            robot.glisiera.setToPosition(pos_glisiera);
        }
        if (b.pressed()) {
            robot.glisiera.modifyPosition(- 100);
        }

        if (pos_down.pressed()) {
            pos_glisiera--;
            if (pos_glisiera < 0)
                pos_glisiera = 0;
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