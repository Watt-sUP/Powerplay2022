package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Mugurel;
import org.firstinspires.ftc.teamcode.hardware.Turela;

@Config
@Autonomous(name = "Gheara Autonom", group = "auto")
public class ClawAutonomous extends LinearOpMode {
    public static int TURELA_TICKS = -1350;

    @Override
    public void runOpMode() throws InterruptedException {
        Mugurel robot = new Mugurel(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(60);
        waitForStart();

        // Get cone
        robot.glisiera.setToPosition(2);
        sleep(600);
        robot.turela.setToPosition(Turela.Position.LEFT);
        sleep(1000);
        robot.glisiera.setToPosition(1);
        sleep(500);
        robot.deget.toggleDeget();
        sleep(200);

        // Drop cone
        robot.glisiera.setToPosition(4);
        sleep(800);
        robot.turela.setToTicks(TURELA_TICKS);
        sleep(600);
        robot.foarfeca.toggleFoarfeca();
        sleep(600);
        robot.deget.toggleDeget();
        sleep(200);

        // Reset bot
        robot.foarfeca.toggleFoarfeca();
        robot.turela.setToPosition(Turela.Position.FRONT);
        sleep(1000);
        robot.glisiera.setToPosition(0);
    }
}
