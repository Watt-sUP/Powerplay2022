package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ConeCommandRight;
import org.firstinspires.ftc.teamcode.commands.helpers.Cone;
import org.firstinspires.ftc.teamcode.commands.subsystems.ColectareSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DetectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TurelaSubsystem;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Config;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.Map;

@com.acmerobotics.dashboard.config.Config
@Autonomous(name = "Autonom 5+1 Dreapta", group = "Autonom")
public class AutonomDreaptaSus extends CommandOpMode {

    public static Cone cone1 = new Cone(Cone.Junctions.Middle, 265);
    public static Cone cone2 = new Cone(Cone.Junctions.Middle, 190);
    public static Cone cone3 = new Cone(Cone.Junctions.Middle, 160);
    public static Cone cone4 = new Cone(Cone.Junctions.Middle, 75);
    public static Cone cone5 = new Cone(Cone.Junctions.Middle, 15);
    public static Cone preload = new Cone(Cone.Junctions.Middle, -1);

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

        TrajectorySequence left_traj = drive.trajectorySequenceBuilder(stack_traj.end())
                .setReversed(true)
                .splineTo(new Vector2d(-11.45, 23.14), Math.toRadians(90.00))
                .setReversed(false)
                .build();

        ColectareSubsystem colectareSystem = new ColectareSubsystem(
                new SimpleServo(hardwareMap, Config.claw, -360, 360),
                new SimpleServo(hardwareMap, Config.foarfeca, -360, 360),
                new SimpleServo(hardwareMap, "PLASTIC", 0, 300)
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

        SequentialCommandGroup autonom = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    drive.setPoseEstimate(stack_traj.start());
                    colectareSystem.toggleClaw();
                }),
                new WaitCommand(300),
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Cone.Junctions.High, new InstantCommand(() -> {
                                glisiereSystem.setToPosition(3);
                                glisiereSystem.lowerUnghi();
                            }));
                            put(Cone.Junctions.Middle, new InstantCommand(() -> {
                                glisiereSystem.setToTicks(ConeCommandRight.midJunction.sliderPosition);
                                glisiereSystem.lowerUnghi();
                            }));
                        }},
                        () -> preload.targetJunction
                ),
                new InstantCommand(() -> drive.followTrajectorySequence(stack_traj)),

                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(Cone.Junctions.High, new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        colectareSystem.retractScissors();
                                        glisiereSystem.setToTicks(ConeCommandRight.highJunction.sliderPosition);
                                        turelaSystem.setToTicks(ConeCommandRight.highJunction.turretPosition);
                                    }),
                                    new WaitUntilCommand(() -> turelaSystem.getTicks() < ConeCommandRight.highJunction.turretPosition * 0.4),
                                    new InstantCommand(() -> colectareSystem.setScissorsPosition(ConeCommandRight.highJunction.scissorsPosition)),
                                    new WaitCommand(200),
                                    new InstantCommand(() -> {
                                        turelaSystem.setToTicks(ConeCommandRight.highJunction.turretPosition, 0.33);
                                        colectareSystem.plastic.turnToAngle(220);
                                    }),
                                    new WaitUntilCommand(() -> turelaSystem.getTicks() < ConeCommandRight.highJunction.turretPosition + 50)
                            ));
                            put(Cone.Junctions.Middle, new SequentialCommandGroup(
                                    new InstantCommand(() -> {
                                        colectareSystem.retractScissors();
                                        turelaSystem.setToTicks(ConeCommandRight.midJunction.turretPosition, 0.45);
                                    }),
                                    new WaitUntilCommand(() -> glisiereSystem.getTicks() > 400 && turelaSystem.getTicks() < -600),
                                    new InstantCommand(() -> colectareSystem.setScissorsPosition(ConeCommandRight.midJunction.scissorsPosition)),
                                    new WaitCommand(200),
                                    new InstantCommand(() -> colectareSystem.plastic.turnToAngle(220)),
                                    new WaitUntilCommand(() -> turelaSystem.getTicks() < ConeCommandRight.midJunction.turretPosition + 50)
                            ));
                        }},
                        () -> preload.targetJunction
                ),
                new WaitCommand(300),
                new InstantCommand(() -> {
                    colectareSystem.setScissorsPosition(0.5);
                    glisiereSystem.setToTicks(ConeCommandRight.stack.sliderPosition);
                }),
                new WaitCommand(250),
                new InstantCommand(() -> {
                    colectareSystem.toggleClaw();
                    colectareSystem.plastic.turnToAngle(0);
                }),

                new ConeCommandRight(cone1, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandRight(cone2, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandRight(cone3, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandRight(cone4, colectareSystem, turelaSystem, glisiereSystem),
                new ConeCommandRight(cone5, colectareSystem, turelaSystem, glisiereSystem),

                new WaitCommand(200),
                new InstantCommand(() -> {
                    colectareSystem.retractScissors();
                    turelaSystem.setToPosition(Direction.FORWARD);
                    glisiereSystem.setToPosition(2);
                }),
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
        schedule(new InstantCommand(() -> {
            FtcDashboard.getInstance().stopCameraStream();
            detectorSystem.close();
        }).andThen(autonom));
    }
}
