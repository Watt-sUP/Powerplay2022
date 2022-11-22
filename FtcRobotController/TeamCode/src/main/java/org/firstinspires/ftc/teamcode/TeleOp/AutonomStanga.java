package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.hardware.Mugurel;
@Disabled
@Autonomous(name = "Autonom Stanga", group = "auto")
public class AutonomStanga extends LinearOpMode{

    private boolean deget = true;
    private Mugurel robot;
    private int pos_glisiera = 0;
    private int pos_turela = 0;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mugurel(hardwareMap);
        frontLeftMotor = hardwareMap.dcMotor.get(Config.left_front);
        backLeftMotor = hardwareMap.dcMotor.get(Config.left_back);
        frontRightMotor = hardwareMap.dcMotor.get(Config.right_front);
        backRightMotor = hardwareMap.dcMotor.get(Config.right_back);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(0);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setTargetPosition(0);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setTargetPosition(0);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setTargetPosition(0);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setPower(0.1);
        frontLeftMotor.setPower(0.1);
        backLeftMotor.setPower(0.1);
        backRightMotor.setPower(0.1);
        waitForStart();

        frontRightMotor.setTargetPosition(1000);
        frontRightMotor.setPower(0.1);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       /*
        frontLeftMotor.setTargetPosition(1000);
        frontLeftMotor.setPower(0.1);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setTargetPosition(1000);
        backLeftMotor.setPower(0.1);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setTargetPosition(-1000);
        backRightMotor.setPower(0.1);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        pos_glisiera = 5;
        robot.glisiera.setToPosition(pos_glisiera);
        sleep(500);

        pos_turela = 1;
        robot.turela.setToPosition(pos_turela);
        sleep(500);

        robot.deget.toggleDeget();
        sleep(500);

        frontRightMotor.setTargetPosition(1000);
        frontRightMotor.setPower(0.1);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setTargetPosition(1000);
        frontLeftMotor.setPower(0.1);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setTargetPosition(1000);
        backLeftMotor.setPower(0.1);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setTargetPosition(1000);
        backRightMotor.setPower(0.1);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);


        frontRightMotor.setTargetPosition(-1000);
        frontRightMotor.setPower(0.1);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setTargetPosition(1000);
        frontLeftMotor.setPower(0.1);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setTargetPosition(-1000);
        backLeftMotor.setPower(0.1);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setTargetPosition(1000);
        backRightMotor.setPower(0.1);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        pos_turela = 3;
        robot.turela.setToPosition(pos_turela);
        sleep(2000);

        pos_glisiera = 3;
        robot.glisiera.setToPosition(pos_glisiera);
        sleep(2000);

        robot.deget.toggleDeget();
        sleep(2000);

        pos_glisiera = 5;
        robot.glisiera.setToPosition(pos_glisiera);
        sleep(2000);

        pos_turela = 1;
        robot.turela.setToPosition(pos_turela);
        sleep(2000);

        frontRightMotor.setTargetPosition(1000);
        frontRightMotor.setPower(0.1);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setTargetPosition(-1000);
        frontLeftMotor.setPower(0.1);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setTargetPosition(1000);
        backLeftMotor.setPower(0.1);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setTargetPosition(-1000);
        backRightMotor.setPower(0.1);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        frontRightMotor.setTargetPosition(-1000);
        frontRightMotor.setPower(0.1);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setTargetPosition(-1000);
        frontLeftMotor.setPower(0.1);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setTargetPosition(-1000);
        backLeftMotor.setPower(0.1);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setTargetPosition(-1000);
        backRightMotor.setPower(0.1);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);

        robot.deget.toggleDeget();
        sleep(100);

        frontRightMotor.setTargetPosition(1000);
        frontRightMotor.setPower(0.1);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setTargetPosition(1000);
        frontLeftMotor.setPower(0.1);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setTargetPosition(1000);
        backLeftMotor.setPower(0.1);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setTargetPosition(1000);
        backRightMotor.setPower(0.1);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(3000);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
*/
        // side note nu stiu daca merge chestia asta e facuta de acasa
    }
}
