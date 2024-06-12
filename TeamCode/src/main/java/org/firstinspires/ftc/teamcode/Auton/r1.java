package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class r1 extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        //Instantiate the drive system
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        //Provide the initial pose
        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270)); //tell the robot where it starts

        //Occupy the initial pose
        drivetrain.setPoseEstimate(startPose);

        //read april tags


        //drive to pixel drop


        //drop pixel


        //drive to backdrop


        //drive to pixel stack, pick up pixels


        //go to backdrop, drop pixels, and park
        TrajectorySequence tragic = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36,36))
                .waitSeconds(2.5)
                .lineToSplineHeading(new Pose2d(-36,45,Math.toRadians(0)))
//                                .splineTo(new Vector2d(12,58),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(18,58),Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(50,36,Math.toRadians(180)))
                .splineTo(new Vector2d(38,7),Math.toRadians(180))
                .splineTo(new Vector2d(-62,12),Math.toRadians(180))

                        .build();

        waitForStart();

        if (!isStopRequested()) {
            drivetrain.followTrajectorySequence(tragic);


        }
    }
}
