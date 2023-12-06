package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous

public class BlueLongVision extends LinearOpMode {

    VisionSubB visub = null;
    VisionPortal myVisionPortal;
    Trajectories trajectories;
    double markerLocation = 300;
    Constants constants = new Constants(this);
    private TrajectorySequence chosenSequence;
    private TrajectorySequence lastSequence;
    private Pose2d backDropGoTO;
    private Pose2d newPose;
    private Pose2d lastPose;


    final int e_tiltPickUp = 0; //The tilt position for picking up a pixel 320 for 5618 and 6494
    final int e_tiltStowed = -400; //The tilt position for moving across the field -30
    final double closed = 0.3;
    final double halfopen = 0.5;
    final double open = 1;
    final int slidePickup = -250;
    final int slideLow = -1300;
    final int slideMed = -1900;
    boolean left = false;
    boolean right = false;
    boolean center = false;
    boolean tryingToScore = false;
    int state = 0;
    int telemetrySequence = 0;

    public void runOpMode() throws InterruptedException {


        // init vision portal
        visub = new VisionSubB(hardwareMap, false);
        constants.init();

        //Instantiate the drive system
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);
        //close claw and lift slide
        constants.claw.setPosition(closed);
        sleep(1000);
        constants.slide_motor.setTargetPosition(slidePickup);
        //Provide the initial pose
        Pose2d blueLongStart = new Pose2d(-39, 60, Math.toRadians(270));
        Pose2d blueShortStart = new Pose2d(12, 60, Math.toRadians(270));
        Pose2d redLongStart = new Pose2d(-39, -60, Math.toRadians(90));
        Pose2d redShortStart = new Pose2d(12, -60, Math.toRadians(90));

        Pose2d startPose = blueShortStart; //tell the robot where it starts

        //Occupy the initial pose
        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence test = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -30, Math.toRadians(180)))
                .forward(5)
                .build();

        TrajectorySequence Left = drivetrain.trajectorySequenceBuilder(startPose)
//             .lineToLinearHeading(new Pose2d(24, -32, Math.toRadians(180)))
//                .back(8)
                .lineToLinearHeading(new Pose2d(12, 32, Math.toRadians(180)))
                .forward(5)
                .build();

        TrajectorySequence Center = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12, 29))
                .build();

        TrajectorySequence Right = drivetrain.trajectorySequenceBuilder(startPose)
//                 .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))
//                .forward(5)
                .lineToLinearHeading(new Pose2d(24, 32, Math.toRadians(180)))
                .back(8)
                .build();





        //read Team Marker Position while waiting for start
        while (!isStarted() && !isStopRequested()) {
            if (visub.foundBLUE()) {
                telemetry.addData("BLUE Found", visub.getTFODX());
                telemetry.update();
                if (visub.getTFODX() > 350) {
                    newPose = Center.end(); // newPose is the end of the first sequence
                    chosenSequence = Center;
                    backDropGoTO = new Pose2d(54, 33, Math.toRadians(180));
                    telemetrySequence = 2;


                } else if (visub.getTFODX() < 300) {
                    newPose = Left.end();
                    chosenSequence = Left;
                    backDropGoTO = new Pose2d(54, 40, Math.toRadians(180));
                    telemetrySequence = 1;


                } else {
                    newPose = Right.end();
                    chosenSequence = Right;
                    backDropGoTO = new Pose2d(54, 27, Math.toRadians(180));
                    telemetrySequence = 3;

                }
            }
            telemetry.addData("BLUE Found", visub.getTFODX());

        }


        telemetry.addData("Chosen Sequence", telemetrySequence);
        telemetry.update();



        TrajectorySequence leftGoToBackdrop = drivetrain.trajectorySequenceBuilder(newPose)
                .lineToLinearHeading(backDropGoTO)

                .build();

        TrajectorySequence centerGoToBackdrop = drivetrain.trajectorySequenceBuilder(newPose)
                .lineToLinearHeading(backDropGoTO)
                .build();

        TrajectorySequence rightGoToBackdrop = drivetrain.trajectorySequenceBuilder(newPose)
                .lineToLinearHeading(backDropGoTO)
                .build();

        if (telemetrySequence == 3) {
            lastSequence = rightGoToBackdrop;
            lastPose = rightGoToBackdrop.end();
        }
        else if (telemetrySequence == 2) {
            lastSequence = centerGoToBackdrop;
            lastPose = centerGoToBackdrop.end();

        }
        else {
            lastSequence = leftGoToBackdrop;
            lastPose = leftGoToBackdrop.end();
        }

        TrajectorySequence goPark = drivetrain.trajectorySequenceBuilder(lastPose)
                .forward(5)
                .lineTo(new Vector2d(43,60))
                .lineTo(new Vector2d(63,60))

                .build();





//*********************************** START IS PRESSED ********************************************
        if (!isStopRequested()) {

            // drive your chosen sequence
            drivetrain.followTrajectorySequence(chosenSequence);
            //drop one pixel
            constants.claw.setPosition(halfopen);
            sleep(500);
            constants.e_tilt.setTargetPosition(e_tiltStowed);
            sleep(500);

//            // drive to the back drop
//            constants.claw.setPosition(closed);
//            sleep(500);
//            drivetrain.followTrajectorySequence(lastSequence);
//
//            // score yellow pixel
//            letsScore();
//
//            //go park
//            drivetrain.followTrajectorySequence(goPark);
        }
    }


    private void letsScore() {
        if (state == 0) {
            constants.e_tilt.setTargetPosition(e_tiltStowed);
            constants.claw.setPosition(closed);
            state++;
        }
        if (state == 1) {
            constants.slide_motor.setTargetPosition(-1800);
            state++;
        }
        //wait for slide to clear
        if (state == 2) {
            telemetry.addData("Debug", "Entering state 1");
            telemetry.update();
            while (opModeIsActive() && constants.slide_motor.getCurrentPosition() > -1700) {
                //wait
            }
            state++;
        }
        //rotate p_tilt
        if (state == 3) {
            telemetry.addData("Debug", "Entering state 2");
            telemetry.update();
            constants.p_tilt.setPosition(1);
            sleep(500);
            state++;
        }
        //release the claw
        if (state == 4) {
            telemetry.addData("Debug", "Entering state 3");
            telemetry.update();
            constants.claw.setPosition(halfopen);
            sleep(400);
            constants.claw.setPosition(open);
            sleep(100);
            state++;
        }
        //close the claw
        if (state == 5) {
            telemetry.addData("Debug", "Entering state 4");
            telemetry.update();
            constants.claw.setPosition(closed);
            sleep(250); //wait for the claw to close
            state++;
        }
        //rotate p_tilt
        if (state == 6) {
            telemetry.addData("Debug", "Entering state 5");
            telemetry.update();
            constants.p_tilt.setPosition(0);
            sleep(700);
            state++;
        }
        //return slide to zero
        if (state == 7) {
            telemetry.addData("Debug", "Entering state 6");
            telemetry.update();
            constants.slide_motor.setTargetPosition(slidePickup);
            sleep(700);
            state++;
        }
        if (state == 8) {
            telemetry.addData("Debug", "Entering state 7");
            telemetry.update();
            constants.claw.setPosition(open);
            sleep(700);
            state = 0;
        }

    }
}
