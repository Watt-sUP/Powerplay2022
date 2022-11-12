package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Config;

@Autonomous(name = "Parcare stanga", group = "auto")
public class parcar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get(Config.left_front);
        DcMotor backLeftMotor = hardwareMap.dcMotor.get(Config.left_back);
        DcMotor frontRightMotor = hardwareMap.dcMotor.get(Config.right_front);
        DcMotor backRightMotor = hardwareMap.dcMotor.get(Config.right_back);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        frontRightMotor.setPower(0.3);
        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(-0.3);
        sleep(3200);

    }


}
