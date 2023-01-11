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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-37, -61, Math.toRadians(90)))
                        .setConstraints(
                                SampleMecanumDrive.getVelocityConstraint(45, 5.315093760821346, 10.513),
                                SampleMecanumDrive.getAccelerationConstraint(45)
                        )
                        .setTurnConstraint(4, 4)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-45, -13), Math.toRadians(170))
                        .resetConstraints()
                        .resetTurnConstraint()
                        .build());

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}