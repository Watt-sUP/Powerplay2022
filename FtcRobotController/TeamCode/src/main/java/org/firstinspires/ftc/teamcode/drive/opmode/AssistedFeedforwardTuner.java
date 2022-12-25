package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.text.DecimalFormat;
import java.util.Hashtable;
import java.util.Objects;

/*
 * This routine is designed to tune the open-loop feedforward coefficients. Although it may seem unnecessary,
 * tuning these coefficients is just as important as the positional parameters. Like the other
 * manual tuning routines, this op mode relies heavily upon the dashboard. To access the dashboard,
 * connect your computer to the RC's WiFi network. In your browser, navigate to
 * https://192.168.49.1:8080/dash if you're using the RC phone or https://192.168.43.1:8080/dash if
 * you are using the Control Hub. Once you've successfully connected, start the program, and your
 * robot will begin moving forward and backward according to a motion profile. Your job is to graph
 * the velocity errors over time and adjust the feedforward coefficients. Once you've found a
 * satisfactory set of gains, add them to the appropriate fields in the DriveConstants.java file.
 *
 * Pressing Y/Δ (Xbox/PS4) will pause the tuning process and enter driver override, allowing the
 * user to reset the position of the bot in the event that it drifts off the path.
 * Pressing B/O (Xbox/PS4) will cede control back to the tuning process.
 */
@Config
@Autonomous(group = "drive")
public class AssistedFeedforwardTuner extends LinearOpMode {
    public static double DISTANCE = 72; // in
    public static double CONSTANT_TOLERANCE = 1.0, ACCEL_TOLERANCE = 1.5;
    public static kA_values values = new kA_values(-1, -1);
    private double middle = (values.low + values.high) / 2;
    private boolean use_kA = true;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private SampleMecanumDrive drive;
    private final DecimalFormat df = new DecimalFormat("0.00");

    double last_target_velo = 0.0;
    double accel_sum = 0.0, accel_count = 0.0, accel_average = 0.0;
    double constant_sum = 0.0, constant_count = 0.0, constant_average = 0.0;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    enum Change {
        INCREASE,
        DECREASE,
        MAINTAIN
    }

    Hashtable<Change, String> str_dict = new Hashtable<>();

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }

    @Override
    public void runOpMode() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        str_dict.put(Change.INCREASE, "increased");
        str_dict.put(Change.DECREASE, "decreased");
        str_dict.put(Change.MAINTAIN, "maintained");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        Mode mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Press start to initialize the tuning process.");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("Mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = 0.0;

                    if (values.high == -1 || values.low == -1)
                        targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
                    else {
                        use_kA = false;
                        targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, middle, kStatic);
                    }
                    drive.setDrivePower(new Pose2d(targetPower, 0, 0));
                    drive.updatePoseEstimate();

                    Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
                    double currentVelo = poseVelo.getX();

                    if (motionState.getV() == last_target_velo)
                    {
                        constant_count++;
                        constant_sum += Math.abs(motionState.getV() - currentVelo);
                        constant_average = constant_sum / constant_count;
                    }

                    else if (motionState.getV() > last_target_velo)
                    {
                        accel_count++;
                        accel_sum += (motionState.getV() - currentVelo);
                        accel_average = accel_sum / accel_count;
                    }

                    else if (motionState.getV() < last_target_velo)
                    {
                        accel_count++;
                        accel_sum += (currentVelo - motionState.getV());
                        accel_average = accel_sum / accel_count;
                    }

                    last_target_velo = motionState.getV();

                    // update telemetry
                    telemetry.clearAll();
                    telemetry.addData("Current kA:", use_kA ? kA : middle);
                    telemetry.addData("Target Velocity", df.format(motionState.getV()));
                    telemetry.addData("Measured Velocity", df.format(currentVelo));
                    telemetry.addData("Current Absolute Error", df.format(Math.abs(motionState.getV() - currentVelo)));
                    telemetry.addData("Average Constant Velocity Error", df.format(constant_average));
                    telemetry.addData("Average Acceleration Error", df.format(accel_average));
                    break;

                case DRIVER_MODE:
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                        resetAccel();
                        resetConstants();
                        last_target_velo = 0.0;

                        if (!use_kA)
                            middle = (values.low + values.high) / 2;
                    }

                    telemetry.clearAll();
                    telemetry.addLine("The constant average was calculated using " + constant_count + "samples.\nThe acceleration average was calculated using " + accel_count + "samples.");
                    telemetry.addData("Final Average Constant Velocity Error", df.format(constant_average));
                    telemetry.addData("Final Average Acceleration Error", df.format(accel_average));

                    Change change = Change.MAINTAIN;
                    if (constant_average > CONSTANT_TOLERANCE)
                        change = Change.DECREASE;
                    else if (Math.abs(accel_average) > ACCEL_TOLERANCE)
                        change = Change.INCREASE;

                    if (use_kA) {
                        if (change == Change.INCREASE || change == Change.DECREASE)
                            telemetry.addLine("\nThe kA value should be " + str_dict.get(change) + ".");

                        else if (change == Change.MAINTAIN) {
                            telemetry.addLine("\nThe kA value should be" + str_dict.get(Change.MAINTAIN) + ". Make sure you update the kA value in your DriveConstants file.");
                            telemetry.addData("kA Value", kA);
                        }
                    }

                    else {
                        telemetry.addLine("The current kA value should be " + str_dict.get(change) + ".");
                        if (change == Change.MAINTAIN)
                            telemetry.addLine("The kA value has been found. Paste the following kA in your code: " + middle);
                        else if (change == Change.INCREASE) {
                            values.low = middle;
                            telemetry.addLine("The new kA value will be set to " + ((values.low + values.high) / 2));
                        }
                        else if (change == Change.DECREASE) {
                            values.high = middle;
                            telemetry.addLine("The new kA value will be set to " + ((values.low + values.high) / 2));
                        }
                    }

                    drive.setWeightedDrivePower(new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    break;
            }

            telemetry.update();
        }
    }

    void resetConstants() {
        constant_count = 0.0;
        constant_sum = 0.0;
        constant_average = 0.0;
    }

    void resetAccel() {
        accel_count = 0.0;
        accel_sum = 0.0;
        accel_average = 0.0;
    }

    public static class kA_values {
        double low;
        double high;

        public kA_values(double low, double high)
        {
            this.low = low;
            this.high = high;
        }
    }
}