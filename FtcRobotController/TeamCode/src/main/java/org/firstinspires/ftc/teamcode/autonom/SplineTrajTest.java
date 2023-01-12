package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Colectare;
import org.firstinspires.ftc.teamcode.hardware.Foarfeca;
import org.firstinspires.ftc.teamcode.hardware.Glisiere;
import org.firstinspires.ftc.teamcode.hardware.Turela;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "SplineTrajTest", group = "auto")
public class SplineTrajTest extends LinearOpMode {

    public static double STRAFE = 6, FORWARD = 50;
    public static double FOARFECA_POS = 0.72, TURELA_TICKS = -500;
    public static double SPEED_LIMIT = 40;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(90));

        Foarfeca foarfeca = new Foarfeca(hardwareMap);
        Colectare colectare = new Colectare(hardwareMap);
        Glisiere glis = new Glisiere(hardwareMap);
        Turela turela = new Turela(hardwareMap);

        drive.setPoseEstimate(startPose);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .setConstraints(
                        SampleMecanumDrive.getVelocityConstraint(SPEED_LIMIT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(SPEED_LIMIT)
                )
                .forward(FORWARD)
                .strafeLeft(STRAFE)
                .addTemporalMarker(-0.25, () -> {
                    glis.setToPosition(4);
                })
                .waitSeconds(0.75)
                .addTemporalMarker(() -> {
                    turela.setToTicks((int) TURELA_TICKS);
                    foarfeca.foarfeca.setPosition(FOARFECA_POS);
                })
                .waitSeconds(2)
                .addTemporalMarker(colectare::toggleDeget)
                .waitSeconds(0.75)
                .addTemporalMarker(() -> {
                    turela.setToPosition(Turela.Position.FRONT);
                    foarfeca.desface();
                    glis.setToPosition(0);
                })
                .waitSeconds(6)
                .resetConstraints()
                .build();

        waitForStart();

        colectare.toggleDeget();
        sleep(500);
        glis.setToPosition(1);
        drive.followTrajectorySequence(traj);
    }
}
