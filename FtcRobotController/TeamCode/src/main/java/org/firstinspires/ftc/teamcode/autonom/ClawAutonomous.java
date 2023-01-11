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

        // Section: Get cone
        // FIXME: Find claw grab timing
        // TODO: Determine ticks for each cone
        robot.glisiera.setToPosition(2);
        sleep(600);
        robot.turela.setToPosition(Turela.Position.LEFT);
        sleep(1000);
        robot.glisiera.setToPosition(1);
        sleep(500);
        robot.deget.toggleDeget();
        sleep(200);

        // Section: Drop cone
        // TODO: Determine turela ticks and custom scissors position
        robot.glisiera.setToPosition(4);
        sleep(800);
        robot.turela.setToTicks(TURELA_TICKS);
        sleep(600);
        robot.foarfeca.toggleFoarfeca();
        sleep(600);
        robot.deget.toggleDeget();
        sleep(200);

        // Section: Reset bot to starting position
        robot.foarfeca.strange();
        robot.turela.setToPosition(Turela.Position.FRONT);
        sleep(1000);
        robot.glisiera.setToPosition(0);
        sleep(1000);
    }
}
