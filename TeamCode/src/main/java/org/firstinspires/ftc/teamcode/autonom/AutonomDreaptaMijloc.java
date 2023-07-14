package org.firstinspires.ftc.teamcode.autonom;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ConeCommandMidRight;
import org.firstinspires.ftc.teamcode.commands.ScanPoleCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DetectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.SensorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Cone;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;

@com.acmerobotics.dashboard.config.Config
@Autonomous(name = "Autonom 5+1 Dreapta (Mijloc)", group = "Autonom")
public class AutonomDreaptaMijloc extends CommandOpMode {

    public static Cone cone1 = new Cone(265, 765, -450, 0.55, 0.58);
    public static Cone cone2 = new Cone(190, 765, -450, 0.55, 0.58);
    public static Cone cone3 = new Cone(115, 765, -450, 0.55, 0.58);
    public static Cone cone4 = new Cone(40, 765, -450, 0.55, 0.58);
    public static Cone cone5 = new Cone(15, 765, -450, 0.55, 0.58);
    public static Cone preload = new Cone(-1, -1, -400, -1, 0.61);

    @Override
    public void initialize() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectorySequence stack_traj = drive.trajectorySequenceBuilder(new Pose2d(-34.76, 63.89, Math.toRadians(-90.00)))
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40)
                )
                .splineTo(new Vector2d(-35.31, 26.46), Math.toRadians(-90.00))
                .splineTo(new Vector2d(-47.00, 12.45), Math.toRadians(180.00))
                .resetConstraints()
                .build();

        TrajectorySequence right_traj = drive.trajectorySequenceBuilder(stack_traj.end())
                .splineTo(new Vector2d(-60.75, 34.76), Math.toRadians(90.00))
                .back(5)
                .build();

        TrajectorySequence middle_traj = drive.trajectorySequenceBuilder(stack_traj.end())
                .setReversed(true)
                .splineTo(new Vector2d(-35.12, 36.23), Math.toRadians(90.00))
                .setReversed(false)
                .build();

        TrajectorySequence left_traj = drive.trajectorySequenceBuilder(new Pose2d(-47.00, 12.45, Math.toRadians(180.00)))
                .setReversed(true)
                .splineTo(new Vector2d(-11.89, 20.74), Math.toRadians(90.00))
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-11.89, 35.86))
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
        FtcDashboard.getInstance().startCameraStream(detectorSystem.getCamera(), 0);

        register(glisiereSystem);
        register(turelaSystem);
        register(colectareSystem);
        register(detectorSystem);
        register(sensorSystem);

        SequentialCommandGroup autonom = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> drive.setPoseEstimate(stack_traj.start())),
                        new InstantCommand(colectareSystem::toggleClaw)
                ),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new InstantCommand(() -> glisiereSystem.setToPosition(3)),
                        new InstantCommand(glisiereSystem::raiseUnghi)
                ),
                new InstantCommand(() -> drive.followTrajectorySequence(stack_traj)),
                new ParallelCommandGroup(
                        new InstantCommand(colectareSystem::retractScissors),
                        new InstantCommand(() -> turelaSystem.setToTicks(preload.stickPos, 0.8)),

                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                                new ScanPoleCommand(turelaSystem, sensorSystem, new Pair<>(-425, -700), 50.0),
                                new WaitUntilCommand(() -> !turelaSystem.isBusy()),
                                new InstantCommand(() -> colectareSystem.setScissorsPosition(preload.stickScissors)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> glisiereSystem.setToPosition(2)),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new InstantCommand(glisiereSystem::lowerUnghi)
                                        )
                                )
                        )
                ),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() < 1250 * 0.8),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(100),

                new ConeCommandMidRight(cone1, colectareSystem, turelaSystem, glisiereSystem, sensorSystem),
                new ConeCommandMidRight(cone2, colectareSystem, turelaSystem, glisiereSystem, sensorSystem),
                new ConeCommandMidRight(cone3, colectareSystem, turelaSystem, glisiereSystem, sensorSystem),
                new ConeCommandMidRight(cone4, colectareSystem, turelaSystem, glisiereSystem, sensorSystem),
                new ConeCommandMidRight(cone5, colectareSystem, turelaSystem, glisiereSystem, sensorSystem),

                new ParallelCommandGroup(
                        new InstantCommand(() -> colectareSystem.setScissorsPosition(0.3)),
                        new InstantCommand(() -> turelaSystem.setToTicks(825)),
                        new InstantCommand(() -> glisiereSystem.setToPosition(2))
                ),
                new SelectCommand(
                        new HashMap<Object, Command>() {
                            {
                                put(-1, new InstantCommand(() -> drive.followTrajectorySequence(left_traj)));
                                put(0, new InstantCommand(() -> drive.followTrajectorySequence(left_traj)));
                                put(1, new InstantCommand(() -> drive.followTrajectorySequence(middle_traj)));
                                put(2, new InstantCommand(() -> drive.followTrajectorySequence(right_traj)));
                            }
                        },
                        () -> detectorSystem.lastDetection
                ),
                new InstantCommand(() -> glisiereSystem.setToPosition(0))
        );

        while (!isStarted()) {
            Map<String, Integer> detection = detectorSystem.getDetection();
            telemetry.addData("Last Detection ID", (detectorSystem.lastDetection == -1) ? "None" : (detectorSystem.lastDetection + 1));
            if (detection != null) {
                telemetry.addData("Detection X", detection.get("x"));
                telemetry.addData("Detection Y", detection.get("y"));
            }
            telemetry.update();
        }

        schedule(new RunCommand(() -> {
            telemetry.addData("Sensor Distance", sensorSystem.getDistance());
            telemetry.update();
        }));

        schedule(new InstantCommand(() -> {
            FtcDashboard.getInstance().stopCameraStream();
            detectorSystem.close();
        }).andThen(autonom));
    }
}
