package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ConeCommandHigh;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DetectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Cone;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;

@com.acmerobotics.dashboard.config.Config
@Autonomous
public class AutonomStangaSus extends CommandOpMode {

    public static Cone cone1 = new Cone(300, -875, 950, 0.5, 0.6);
    public static Cone cone2 = new Cone(225, -875, 950, 0.5, 0.6);
    public static Cone cone3 = new Cone(150, -875, 950, 0.5, 0.6);
    public static Cone cone4 = new Cone(75, -875, 950, 0.5, 0.6);
    public static Cone cone5 = new Cone(0, -875, 950, 0.5, 0.6);

    public static int DROP_TICKS = 725;
    public static Cone preload = new Cone(-1, -1, 950, -1, 0.6);

    @Override
    public void initialize() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-34, -61, Math.toRadians(90));

        Trajectory stack_traj = drive.trajectoryBuilder(startPose)
                .splineTo(
                        new Vector2d(-46, -12), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        TrajectorySequence left_traj = drive.trajectorySequenceBuilder(stack_traj.end())
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(new Vector2d(-60, -35), Math.toRadians(270))
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-60, -27, Math.toRadians(270)))
                .build();
        TrajectorySequence middle_traj = drive.trajectorySequenceBuilder(stack_traj.end())
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .setReversed(true)
                .splineTo(new Vector2d(-34, -32), Math.toRadians(270))
                .build();
        TrajectorySequence right_traj = drive.trajectorySequenceBuilder(stack_traj.end())
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .setReversed(true)
                .lineTo(new Vector2d(-26, -12))
                .splineTo(new Vector2d(-11, -32), Math.toRadians(270))
                .build();

        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.claw, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360)
        );
        GlisiereSubsystem glisiereSystem = new GlisiereSubsystem(
                hardwareMap.dcMotor.get(Config.glisiera),
                hardwareMap.dcMotor.get(Config.glisiera1)
        );
        TurelaSubsystem turelaSystem = new TurelaSubsystem(new Motor(hardwareMap, Config.turela));
        DetectorSubsystem detectorSystem = new DetectorSubsystem(hardwareMap, 0, 1, 2);

        register(glisiereSystem);
        register(turelaSystem);
        register(colectareSystem);
        register(detectorSystem);

        SequentialCommandGroup autonom = new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPoseEstimate(startPose)),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(400),
                new InstantCommand(() -> glisiereSystem.setToPosition(3)),
                new InstantCommand(() -> drive.followTrajectory(stack_traj)),
                new InstantCommand(() -> glisiereSystem.setToPosition(4), glisiereSystem),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() > 800),
                new InstantCommand(colectareSystem::retractScissors),
                new ParallelCommandGroup(
                        new InstantCommand(() -> turelaSystem.setToTicks(preload.stickPos, 0.8)),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > DROP_TICKS),
                                new InstantCommand(() -> colectareSystem.setScissorsPosition(preload.stickScissors)),
                                new WaitCommand(150),
                                new InstantCommand(() -> glisiereSystem.setToPosition(2))
                        )
                ),
                new WaitCommand(250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(100),
                new ConeCommandHigh(cone1, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandHigh(cone2, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandHigh(cone3, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandHigh(cone4, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandHigh(cone5, colectareSystem, turelaSystem, glisiereSystem),
                new ParallelCommandGroup(
                        new InstantCommand(colectareSystem::retractScissors),
                        new InstantCommand(() -> glisiereSystem.setToPosition(2)),
                        new InstantCommand(() -> turelaSystem.setToTicks(-825))
                ),
                new SelectCommand(
                        new HashMap<Object, Command>() {
                            {
                                put(-1, new InstantCommand(() -> drive.followTrajectorySequence(right_traj)));
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

            if (isStopRequested())
                return;

            Map<String, Integer> detection = detectorSystem.getDetection();
            telemetry.addData("Last Detection ID", detectorSystem.lastDetection);
            if (detection != null) {
                telemetry.addData("Detection X", detection.get("x"));
                telemetry.addData("Detection Y", detection.get("y"));
            }
            telemetry.update();
        }

        schedule(new InstantCommand(detectorSystem::close).andThen(autonom));
    }
}
