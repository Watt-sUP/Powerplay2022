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
                .setConstraints(56.25590416793869, 56.25590416793869, 2.997602939605713, 2.997602939605713, 14.76)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34, -61, Math.toRadians(90)))
                        .setConstraints(
                                SampleMecanumDrive.getVelocityConstraint(40, 2.997602939605713, 14.76),
                                SampleMecanumDrive.getAccelerationConstraint(40)
                        )
                        .splineTo(new Vector2d(-50, -12), Math.toRadians(180))
//                        .setTangent(Math.toRadians(180))
                        .setReversed(true)
                        .lineTo(new Vector2d(-26, -12))
                        .splineTo(new Vector2d(-11, -32), Math.toRadians(270))
                        .resetConstraints()
                        .resetTurnConstraint()
                        .build());

        mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .addEntity(bot)
                .start();
    }
}