package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ConeCommandMidLeft;
import org.firstinspires.ftc.teamcode.commands.ConeCommandSafe;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DetectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Cone;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Map;

@com.acmerobotics.dashboard.config.Config
@Autonomous
public class AutonomSiguranta extends CommandOpMode {

    public static Cone preload = new Cone(-1, -1, 520, 0, 0.62);
    public static Cone cone1 = new Cone(315, -810, 515, 0.5, 0.52);
    public static Cone cone2 = new Cone(240, -810, 515, 0.5, 0.52);
    public static Cone cone3 = new Cone(165, -810, 515, 0.5, 0.52);
    public static Cone cone4 = new Cone(90, -810, 515, 0.5, 0.52);
    public static Cone cone5 = new Cone(15, -810, 515, 0.5, 0.52);


    public static int DROP_TICKS = 375, PRELOAD_OFFSET = 50;

    @Override
    public void initialize() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence stack_traj = drive.trajectorySequenceBuilder(new Pose2d(-35.68, -64.07, Math.toRadians(90.00)))
                .splineTo(
                        new Vector2d(-49.69, -12.45), Math.toRadians(180.00),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40)
                )
                .build();

        Trajectory go_to_stick = drive.trajectoryBuilder(stack_traj.end())
                .lineToConstantHeading(new Vector2d(-23.88, -12.45))
                .build();

        Trajectory go_back = drive.trajectoryBuilder(go_to_stick.end())
                .lineToConstantHeading(new Vector2d(stack_traj.end().getX(), stack_traj.end().getY()))
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
        FtcDashboard.getInstance().startCameraStream(detectorSystem.getCamera(), 0);

        register(glisiereSystem);
        register(turelaSystem);
        register(colectareSystem);
        register(detectorSystem);

        ConeCommandSafe cmd1 = new ConeCommandSafe(cone1, colectareSystem, turelaSystem, glisiereSystem);
        ConeCommandSafe cmd2 = new ConeCommandSafe(cone2, colectareSystem, turelaSystem, glisiereSystem);
        ConeCommandSafe cmd3 = new ConeCommandSafe(cone3, colectareSystem, turelaSystem, glisiereSystem);
        ConeCommandSafe cmd4 = new ConeCommandSafe(cone4, colectareSystem, turelaSystem, glisiereSystem);


        SequentialCommandGroup autonom = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> drive.setPoseEstimate(stack_traj.start())),
                        new InstantCommand(colectareSystem::toggleClaw)
                ),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new InstantCommand(() -> glisiereSystem.setToTicks(1500)),
                        new InstantCommand(() -> glisiereSystem.turnUnghiToAngle(180))
                ),
                new InstantCommand(() -> drive.followTrajectorySequence(stack_traj)),
                new ParallelCommandGroup(
                        new InstantCommand(colectareSystem::retractScissors),
                        new InstantCommand(() -> turelaSystem.setToTicks(preload.stickPos, 0.8)),

                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > DROP_TICKS + PRELOAD_OFFSET && glisiereSystem.getTicks() > 1000),
                                new InstantCommand(() -> colectareSystem.setScissorsPosition(preload.stickScissors)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> turelaSystem.setToTicks(preload.stickPos, 0.33)),
                                        new InstantCommand(() -> glisiereSystem.setToPosition(2)),
                                        new SequentialCommandGroup(
                                                new WaitCommand(100),
                                                new InstantCommand(glisiereSystem::lowerUnghi)
                                        )
                                )
                        )
                ),
                new WaitUntilCommand(() -> glisiereSystem.getTicks() < 1250),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(100),
                cmd1.collectCone(),
                new InstantCommand(() -> drive.followTrajectory(go_to_stick)),
                cmd1.depositCone(),
                new InstantCommand(() -> drive.followTrajectoryAsync(go_back)),
                new ParallelRaceGroup(
                        new RunCommand(drive::update)
                                .interruptOn(() -> !drive.isBusy()),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() < -37)
                                .andThen(cmd2.collectCone())
                ),
                cmd2.collectCone(),
                new InstantCommand(() -> drive.followTrajectory(go_to_stick)),
                cmd2.depositCone(),
                new InstantCommand(() -> drive.followTrajectoryAsync(go_back)),
                new ParallelRaceGroup(
                        new RunCommand(drive::update)
                                .interruptOn(() -> !drive.isBusy()),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() < -37)
                                .andThen(cmd3.collectCone())
                ),
                cmd3.collectCone(),
                new InstantCommand(() -> drive.followTrajectory(go_to_stick)),
                cmd3.depositCone(),
                new InstantCommand(() -> drive.followTrajectoryAsync(go_back)),
                new ParallelRaceGroup(
                        new RunCommand(drive::update)
                                .interruptOn(() -> !drive.isBusy()),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() < -37)
                                .andThen(cmd4.collectCone())
                ),
                new ConeCommandMidLeft(cone4, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandMidLeft(cone5, colectareSystem, turelaSystem, glisiereSystem)
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

        schedule(new InstantCommand(() -> {
            FtcDashboard.getInstance().stopCameraStream();
            detectorSystem.close();
        }).andThen(autonom));
    }
}
