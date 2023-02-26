package com.example.meepmeep;

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
                .setDimensions(14.76, 15.74)
                .setConstraints(56.25590416793869, 56.25590416793869, 2.997602939605713, 2.997602939605713, 14.76)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-35.68, -64.07, Math.toRadians(90.00)))
                        .setConstraints(
                                SampleMecanumDrive.getVelocityConstraint(40, 2.5, 14.14),
                                SampleMecanumDrive.getAccelerationConstraint(40)
                        )
                        .splineTo(new Vector2d(-47, -12.45), Math.toRadians(180.00))
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(-21.85, -12.45))
                        .splineTo(new Vector2d(-12.45, -37), Math.toRadians(270.00))
                        .setReversed(false)
                        .resetConstraints()
                        .resetTurnConstraint()
                        .build());

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}