package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ConeCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DetectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Cone;
import org.firstinspires.ftc.teamcode.hardware.Config;

import java.util.Map;

@Autonomous(name = "Autonom 5+1 Stanga")
public class AutonomStangaBun extends CommandOpMode {

    private final Cone cone1 = new Cone(300, -875, 475, 0.52, 0.55);
    private final Cone cone2 = new Cone(225, -875, 465, 0.52, 0.55);
    private final Cone cone3 = new Cone(150, -875, 465, 0.52, 0.55);
    private final Cone cone4 = new Cone(75, -875, 465, 0.52, 0.55);
    private final Cone cone5 = new Cone(0, -875, 465, 0.52, 0.55);

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
                new ParallelCommandGroup(
                        new InstantCommand(() -> colectareSystem.setScissorsPosition(0.55)),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> turelaSystem.modifyByTicks(475, 0.8)),
                                new WaitUntilCommand(() -> turelaSystem.getTicks() > 400),
                                new InstantCommand(() -> glisiereSystem.setToPosition(2))
                        )
                ),
                new WaitCommand(100),
                new InstantCommand(colectareSystem::toggleClaw),
                new WaitCommand(100),
                new ConeCommand(cone1, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommand(cone2, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommand(cone3, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommand(cone4, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommand(cone5, colectareSystem, turelaSystem, glisiereSystem)
        );

        while (!isStarted()) {
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
