package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;

@Autonomous(name = "Autonom Stanga", group = "auto")
public class AutonomStanga extends LinearOpMode{

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

        frontRightMotor.setPower(0.3);
        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(0.3);
        sleep(3000);

        pos_glisiera = 5;
        robot.glisiera.setToPosition(pos_glisiera);
        sleep(100);

        pos_turela = 1;
        robot.turela.setToPosition(pos_turela);
        sleep(100);

        robot.deget.toggleDeget();
        sleep(100);

        frontRightMotor.setPower(-0.3);
        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(-0.3);
        backRightMotor.setPower(-0.3);
        sleep(1000);

        frontRightMotor.setPower(0.3);
        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(-0.3);
        sleep(1000);

        pos_turela = 3;
        robot.turela.setToPosition(pos_turela);
        sleep(100);

        pos_glisiera = 3;
        robot.glisiera.setToPosition(pos_glisiera);
        sleep(100);

        robot.deget.toggleDeget();
        sleep(100);

        pos_glisiera = 5;
        robot.glisiera.setToPosition(pos_glisiera);
        sleep(100);

        pos_turela = 1;
        robot.turela.setToPosition(pos_turela);
        sleep(100);

        frontRightMotor.setPower(-0.3);
        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(-0.3);
        backRightMotor.setPower(0.3);
        sleep(1000);

        frontRightMotor.setPower(0.3);
        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        backRightMotor.setPower(0.3);
        sleep(1000);

        robot.deget.toggleDeget();
        sleep(100);

        frontRightMotor.setPower(-0.3);
        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(-0.3);
        backRightMotor.setPower(-0.3);
        sleep(1000);

        // side note nu stiu daca merge chestia asta e facuta de acasa
    }
}
