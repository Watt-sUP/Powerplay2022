package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "SplineTrajTest", group = "auto")
public class SplineTrajTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -61, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(45, 5.315093760821346, 10.513),
                        SampleMecanumDrive.getAccelerationConstraint(45)
                )
                .splineToSplineHeading(new Pose2d(-12, -23, Math.toRadians(90)), Math.toRadians(60))
                .addTemporalMarker(0.2, 0, () -> {
                })
                .addTemporalMarker(5, () -> {
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                })
                .addTemporalMarker(6.5, () -> {
                })
                .resetConstraints()
                // .waitSeconds(2)
                .build();

        drive.followTrajectorySequence(traj);
    }
}
