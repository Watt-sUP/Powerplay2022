package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;

@TeleOp(name = "Encoder tester", group = "Testing")
public class TesterEncoder extends LinearOpMode {

    private Mugurel robot;
    @Override
    public void runOpMode() {
        robot = new Mugurel(hardwareMap);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Config.left_front);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Config.left_back);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Config.right_front);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Config.right_back);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {

            if(gamepad2.a) {
                frontRightMotor.setPower(0.5);
                frontRightMotor.setTargetPosition(538);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad2.b) {
                frontLeftMotor.setPower(0.5);
                frontLeftMotor.setTargetPosition(538);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad2.y) {
                backLeftMotor.setPower(0.5);
                backLeftMotor.setTargetPosition(538);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            if(gamepad2.x) {
                backRightMotor.setPower(0.5);
                backRightMotor.setTargetPosition(538);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }
    }
}
