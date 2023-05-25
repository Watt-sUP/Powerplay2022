package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;


// This is an example of a more complex path to really test the tuning.

@Disabled
@Config
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    public static int WAIT_TIME = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Foarfeca foarfeca = new Foarfeca(hardwareMap);
        Colectare deget = new Colectare(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(traj);

        sleep(WAIT_TIME);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}
