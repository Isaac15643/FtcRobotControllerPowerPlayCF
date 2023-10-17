package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class linetobackboard extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -66, Math.toRadians(90)); //tell the robot where it starts
        drivetrain.setPoseEstimate(startPose);

        //Roadrunner Guide to Trajectory Options:
        //https://learnroadrunner.com/trajectorybuilder-functions.html#linetolinearheading-endpose-pose2d

        //Trajectory Sequence example:
        TrajectorySequence trajSeq = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36,-24))

                .build();


        waitForStart();

        if (!isStopRequested())
            drivetrain.followTrajectorySequence(trajSeq);

    }
}
