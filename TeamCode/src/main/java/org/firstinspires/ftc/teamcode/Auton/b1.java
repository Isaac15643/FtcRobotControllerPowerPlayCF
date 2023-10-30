package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous

public class b1 extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        int teamMarker = 0; //variable for the location of team marker (1,2,3 = L, C, R)

        //Establish AprilTag detection


        //Instantiate the drive system
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        //Provide the initial pose
        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(270)); //tell the robot where it starts

        //Occupy the initial pose
        drivetrain.setPoseEstimate(startPose);

        //read Team Marker Position


        //drive to pixel drop


        //drop pixel


        //drive to backdrop


        //drive to pixel stack, pick up pixels


        //go to backdrop, drop pixels, and park
        TrajectorySequence tragiccenter = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36,32))
                .waitSeconds(2.5)
                .splineTo(new Vector2d(-28,0),Math.toRadians(0))
                .lineTo(new Vector2d(-10,0))
                .lineTo(new Vector2d(28,20))
                .lineToLinearHeading(new Pose2d(50,33,Math.toRadians(180)))
                .waitSeconds(2.5)
                .lineTo(new Vector2d(22,0))
                .lineTo(new Vector2d(-10,0))
                .lineTo(new Vector2d(-58,10))
                .waitSeconds(2.5)
                .back(5)
                .lineTo(new Vector2d(50,0))

                .build();




//        TrajectorySequence tragicright = drivetrain.trajectorySequenceBuilder(startPose)
//                .lineTo(new Vector2d(-36,32))
//                .turn(Math.toRadians(-90))
//                .waitSeconds(2.5)
//                .splineTo(new Vector2d(-28,10),Math.toRadians(0))
//                .splineTo(new Vector2d(-10,8),Math.toRadians(0))
//                .splineTo(new Vector2d(22,20),Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(50,36,Math.toRadians(180)))
//                .waitSeconds(2.5)
//                .splineTo(new Vector2d(22,10),Math.toRadians(180))
//                .splineTo(new Vector2d(-10,8),Math.toRadians(180))
//                .splineTo(new Vector2d(-57,13),Math.toRadians(180))
//                .waitSeconds(2.5)
//                .back(5)
//                .splineTo(new Vector2d(-10,8),Math.toRadians(0))
//                .splineTo(new Vector2d(22,10),Math.toRadians(0))
//                .splineTo(new Vector2d(60,13),Math.toRadians(0))
//
//
//                .build();



        //        TrajectorySequence tragicleft = drivetrain.trajectorySequenceBuilder(startPose)
//                .lineTo(new Vector2d(-36,32))
//                .turn(Math.toRadians(-90))
//                .waitSeconds(2.5)
//                .splineTo(new Vector2d(-28,10),Math.toRadians(0))
//                .splineTo(new Vector2d(-10,8),Math.toRadians(0))
//                .splineTo(new Vector2d(22,20),Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(50,36,Math.toRadians(180)))
//                .waitSeconds(2.5)
//                .splineTo(new Vector2d(22,10),Math.toRadians(180))
//                .splineTo(new Vector2d(-10,8),Math.toRadians(180))
//                .splineTo(new Vector2d(-57,13),Math.toRadians(180))
//                .waitSeconds(2.5)
//                .back(5)
//                .splineTo(new Vector2d(-10,8),Math.toRadians(0))
//                .splineTo(new Vector2d(22,10),Math.toRadians(0))
//                .splineTo(new Vector2d(60,13),Math.toRadians(0))
//
//
//                .build();

        waitForStart();

        if (!isStopRequested()) {
            drivetrain.followTrajectorySequence(tragiccenter);
//            drivetrain.followTrajectorySequence(tragicright);
            //  drivetrain.followTrajectorySequence(tragicleft);


        }
    }
}
