package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Commands;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
//@Disabled

public class BlueRight extends LinearOpMode {

    private org.firstinspires.ftc.teamcode.drive.Utilities Utilities;
    Constants constants = new Constants(this);
    Commands commands;

    @Override

    public void runOpMode() throws InterruptedException {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        Pose2d BlueLeftStart = new Pose2d(36, 60, Math.toRadians(270));
        Pose2d BlueRightStart = new Pose2d(12, 60, Math.toRadians(270));
        Pose2d RedLeftStart = new Pose2d(-36, -63, Math.toRadians(90));
        Pose2d RedRightStart = new Pose2d(12, -60, Math.toRadians(90));

        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(270)); //tell the robot where it starts

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence tragic = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(BlueRightStart)
                .lineTo(new Vector2d(36,36))

                .build();
        waitForStart();

        if (!isStopRequested()) {
            drivetrain.followTrajectorySequence(tragic);

        }
    }
}
