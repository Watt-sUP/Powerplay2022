package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ConeLeftSensorCommand;
import org.firstinspires.ftc.teamcode.commands.ScanPoleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DetectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Cone;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Map;

@Autonomous
@com.acmerobotics.dashboard.config.Config
public class AutonomStangaSusNou extends CommandOpMode {

    public static Cone cone1 = new Cone(340, -1600, -500, 0.8, 0.8);

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence stack = drive.trajectorySequenceBuilder(new Pose2d(-33.83, -64.44, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-34.02, -12.81))
                .strafeRight(5)
                .build();
        drive.setPoseEstimate(stack.start());

        TrajectorySequence right = drive.trajectorySequenceBuilder(stack.end())
                .lineToConstantHeading(new Vector2d(-11.71, -12.81))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(stack.end())
                .lineToConstantHeading(new Vector2d(-57.99, -13.00))
                .build();

        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.claw, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360)
        );
        colectareSystem.setClawPosition(0.25);

        GlisiereSubsystem glisiereSystem = new GlisiereSubsystem(
                hardwareMap.dcMotor.get(Config.glisiera),
                hardwareMap.dcMotor.get(Config.glisiera1),
                new SimpleServo(hardwareMap, Config.ghidaj, 0, 300), false
        );
        TurelaSubsystem turelaSystem = new TurelaSubsystem(new Motor(hardwareMap, Config.turela));
        DetectorSubsystem detectorSystem = new DetectorSubsystem(hardwareMap, 0, 1, 2);
        SensorSubsystem sensorSystem = new SensorSubsystem(hardwareMap, "distance");

        register(glisiereSystem);
        register(turelaSystem);
        register(colectareSystem);
        register(detectorSystem);
        register(sensorSystem);

        SequentialCommandGroup auto = new SequentialCommandGroup(
                // Grab preload
                new ParallelCommandGroup(
                        new InstantCommand(() -> drive.setPoseEstimate(stack.start())),
                        new InstantCommand(colectareSystem::toggleClaw)
                ),
                new WaitCommand(300),
                // Drive
                new ParallelCommandGroup(
                        new InstantCommand(glisiereSystem::raiseUnghi),
                        new InstantCommand(() -> glisiereSystem.setToPosition(3)),
                        new InstantCommand(() -> turelaSystem.setToTicks(-575, 0.5))
                ),
                new InstantCommand(() -> drive.followTrajectorySequence(stack)),
                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                // Score preload
                new ScanPoleCommand(turelaSystem, sensorSystem, ScanPoleCommand.Direction.RIGHT, 20.0),
                new InstantCommand(() -> colectareSystem.setScissorsPosition(0.25)),
                new WaitCommand(200),
                new InstantCommand(() -> {
                    colectareSystem.openClaw();
                    glisiereSystem.lowerUnghi();
                }),
                new ConeLeftSensorCommand(cone1, ConeLeftSensorCommand.Poles.DANGER_HIGH,
                        colectareSystem, turelaSystem, glisiereSystem, sensorSystem)
        );

        while (!isStarted()) {
            if (isStopRequested())
                return;

            Map<String, Integer> detection = detectorSystem.getDetection();
            telemetry.addData("Last Detection ID", (detectorSystem.lastDetection == -1) ? "None" : (detectorSystem.lastDetection + 1));
            if (detection != null) {
                telemetry.addData("Detection X", detection.get("x"));
                telemetry.addData("Detection Y", detection.get("y"));
            }
            telemetry.update();
        }
        detectorSystem.close();

        schedule(auto.alongWith(new RunCommand(() -> {
            telemetry.addData("Turret Ticks", turelaSystem.getTicks());
            telemetry.addData("Sensor Distance (CM)", sensorSystem.getDistance());
            telemetry.update();
        })));
    }
}
