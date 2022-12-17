package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mm = new MeepMeep(640);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(mm)
                .setConstraints(55.57470216843153, 55.57470216843153, 5.315093760821346, 5.315093760821346, 10.513)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -61, Math.toRadians(90)))
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
                                .lineTo(new Vector2d(-12, -12))
                                .addDisplacementMarker(() -> {
                                })
                                .addTemporalMarker(6.5, () -> {
                                })
                                .resetConstraints()
                                // .waitSeconds(2)
                                .build()
                );

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}