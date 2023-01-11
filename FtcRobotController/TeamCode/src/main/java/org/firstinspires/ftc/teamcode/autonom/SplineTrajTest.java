package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(name = "SplineTrajTest", group = "auto")
public class SplineTrajTest extends LinearOpMode {

    public static double STRAFE = 24, FORWARD = 38;
    public static double SPLINE_END_X = 56, SPLINE_END_Y = -12;

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
                .waitSeconds(1.7)
                .addTemporalMarker(() -> {
                })
                .waitSeconds(0.3)
                .splineTo(new Vector2d(SPLINE_END_X, SPLINE_END_Y), Math.toRadians(0))
                .addTemporalMarker(() -> {
                })
                .resetConstraints()
                .resetTurnConstraint()
                .build();

        drive.followTrajectorySequence(traj);
    }
}
