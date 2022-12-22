package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "SplineTrajTest", group = "auto")
public class SplineTrajTest extends LinearOpMode {

    public static double STRAFE = 24, FORWARD = 38;
    public static Vector2d SPLINE_END = new Vector2d(59, -12);

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(45, 5.315093760821346, 10.513),
                        SampleMecanumDrive.getAccelerationConstraint(45)
                )
                .setTurnConstraint(4, 4)
                .strafeLeft(STRAFE)
                .forward(FORWARD)
                .addTemporalMarker(0.2, 0, () -> {
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                })
                .waitSeconds(0.3)
                .splineTo(SPLINE_END, Math.toRadians(0))
                .addTemporalMarker(() -> {
                })
                .resetConstraints()
                .resetTurnConstraint()
                .build();

        drive.followTrajectorySequence(traj);
    }
}
