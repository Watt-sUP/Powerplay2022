package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;

@Disabled
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

            if(gamepad2.a)
                test_encoder(frontRightMotor);

            if(gamepad2.b)
                test_encoder(frontLeftMotor);

            if(gamepad2.y)
                test_encoder(backLeftMotor);

            if(gamepad2.x)
                test_encoder(backRightMotor);
        }
    }

    public void test_encoder(DcMotor motor)
    {
        motor.setPower(0.5);
        motor.setTargetPosition(538);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
