package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;
import org.firstinspires.ftc.teamcode.hardware.Glisiere;

@Config
//@Disabled
@TeleOp
public class TurelaTuner extends LinearOpMode {
    public static boolean REVERSE = true;
    public static double kP = 12, kD = 0, kI = 0;
    public static int TOLERANCE, TARGET = 2000, GLIS_POS = 2;

    InterpLUT kP_lut, tolerance_lut;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        Foarfeca foarfeca = new Foarfeca(hardwareMap);
        Colectare colectare = new Colectare(hardwareMap);
        Glisiere glisiere = new Glisiere(hardwareMap);
        PIDController pid = new PIDController(kP, kI, kD);

        kP_lut = new InterpLUT();
        tolerance_lut = new InterpLUT();

        kP_lut.add(-2000, 18);
        kP_lut.add(2150, 1.5);

        tolerance_lut.add(-2000, 75);
        tolerance_lut.add(2150, 150);

        kP_lut.createLUT();
        tolerance_lut.createLUT();

        pid.setSetPoint(TARGET);
        pid.setTolerance(TOLERANCE);
        MotorEx turela = new MotorEx(hardwareMap, org.firstinspires.ftc.teamcode.hardware.Config.turela);
        turela.setRunMode(Motor.RunMode.RawPower);
        turela.setInverted(REVERSE);

        glisiere.setToPosition(GLIS_POS);

        telemetry.addData("Current Position", -turela.getCurrentPosition());
        telemetry.addData("Current Velocity", turela.getVelocity());
        telemetry.addData("Target Position", TARGET);
        telemetry.update();

        waitForStart();

        while (!pid.atSetPoint() && !isStopRequested()) {
            double output = pid.calculate(-turela.getCurrentPosition());
            turela.setVelocity(output);
            telemetry.addData("Current Position", -turela.getCurrentPosition());
            telemetry.addData("Target Position", TARGET);
            telemetry.addData("Current Velocity", output);
            telemetry.update();
        }
        turela.stopMotor();
        pid.reset();
    }
}
