package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;
import org.firstinspires.ftc.teamcode.hardware.Glisiere;
import org.firstinspires.ftc.teamcode.hardware.TurelaBeta;

@Config
@Disabled
@TeleOp
public class TurelaTuner extends LinearOpMode {
    // TODO: Tune the Position Coefficient and (optionally kD)
    public static double kP = 0.2, kD = 0.25;
    public boolean USE_kD = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        TurelaBeta tur = new TurelaBeta(hardwareMap);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        Motor turela = tur.turela;

        Glisiere glisiere = new Glisiere(hardwareMap);
        glisiere.setToPosition(3);

        Foarfeca foarfeca = new Foarfeca(hardwareMap);
        Colectare colectare = new Colectare(hardwareMap);

        TurelaBeta.Goal goal = TurelaBeta.Goal.LEFT;

        int goal_ticks = tur.goal_values.getOrDefault(goal, 0);
        turela.set(0.0);
        turela.setTargetPosition(goal_ticks);
        turela.setPositionTolerance(10);

        waitForStart();
        while (opModeIsActive() &&!gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {

            if (kP != turela.getPositionCoefficient())
                turela.setPositionCoefficient(kP);

            telemetry.addData("Current kP:", turela.getPositionCoefficient());
            telemetry.addData("Current encoder position (Ticks):", turela.getCurrentPosition());
            telemetry.addData("Current goal (Ticks):", goal_ticks);
            telemetry.update();

            if (!turela.atTargetPosition())
                turela.set(1.0);
            else {
                turela.stopMotor();
                goal = toggleGoal(goal);
                sleep(1500);
                goal_ticks = tur.goal_values.getOrDefault(goal, 0);
                turela.setTargetPosition(goal_ticks);
                turela.set(0.0);
                turela.setPositionTolerance(15);
            }
        }
    }

    private TurelaBeta.Goal toggleGoal(TurelaBeta.Goal goal) {
        return (goal == TurelaBeta.Goal.LEFT) ? TurelaBeta.Goal.RIGHT : TurelaBeta.Goal.LEFT;
    }
}
