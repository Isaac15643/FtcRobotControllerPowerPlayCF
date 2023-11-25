package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Trajectories {

    MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

    TrajectorySequence b1Left (Pose2d startPose){
        drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-46, 20))
                .build();
        return null;
    }

    public TrajectorySequence b1Center (Pose2d startPose){
        drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 29))
                .build();
        return null;
    }

    public TrajectorySequence b1Right (Pose2d startPose){
        drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-24, 20))
                .build();
        return null;
    }
}
