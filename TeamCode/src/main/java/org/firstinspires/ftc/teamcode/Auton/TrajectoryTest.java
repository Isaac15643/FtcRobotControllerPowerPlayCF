package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
@Disabled
public class TrajectoryTest extends LinearOpMode {
    @Override


    public void runOpMode() throws InterruptedException {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90)); //tell the robot where it starts

        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence tragic = drivetrain.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-58,12),Math.toRadians(180))
//                .turn(Math.toRadians(0))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drivetrain.followTrajectorySequence(tragic);
//        drivetrain.followTrajectorySequence(trajSeq2);

        }
    }
}
