package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous

public class RedShort extends LinearOpMode {

    AprilTagProcessor aprilTag;
    VisionPortal myVisionPortal;
    Trajectories trajectories;
    double markerLocation;
    Constants constants = new Constants(this);
    private TrajectorySequence chosenSequence;


    public void runOpMode() throws InterruptedException {


        // Init the AprilTag processor and Vision Portal
        initAprilTag();

        //Instantiate the drive system
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);
        final double closed = 0.3;
        final double halfopen = 0.525;
        final double open = 1;
        final int slidePickup = -250;
        final int slideLow = -1300;
        final int slideMed = -1900;
        //close claw and lift slide
        constants.claw.setPosition(1);
        constants.slide_motor.setTargetPosition(-180);
        //Provide the initial pose
        Pose2d blueLongStart = new Pose2d(-36, 60, Math.toRadians(270));
        Pose2d blueShortStart = new Pose2d(12, 60, Math.toRadians(270));
        Pose2d redLongStart = new Pose2d(-36, -60, Math.toRadians(90));
        Pose2d redShortStart = new Pose2d(12, -60, Math.toRadians(90));

        Pose2d startPose = redShortStart; //tell the robot where it starts

        //Occupy the initial pose
        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence Left = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12,-36,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(10,-36))
                .addTemporalMarker(() -> constants.claw.setPosition(halfopen))

                .build();

        TrajectorySequence Center = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12, -36))
                .build();

        TrajectorySequence Right = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12, -36))
                .build();

        Pose2d newPose = Left.end(); // We are just giving it a value. The value will be reassigned later

        //read Team Marker Position while waiting for start
        while (!isStarted() && !isStopRequested()) {
            telemetryAprilTag();
            telemetry.addData("Marker Detected at", markerLocation);
            telemetry.update();
            if (markerLocation > 400) {
                newPose = Right.end(); // newPose is the end of the first sequence
                chosenSequence = Right;



                } else if (markerLocation < 300) {
                    newPose = Center.end();
                    chosenSequence = Center;

            } else {
                newPose = Left.end();
                chosenSequence = Left;
            }
        }


        TrajectorySequence goToBackdrop = drivetrain.trajectorySequenceBuilder(newPose)
                .lineTo(new Vector2d(-36,0))
                .build();

//*********************************** START IS PRESSED ********************************************
        if (!isStopRequested()) {
            // Disable the AprilTag processor.
            myVisionPortal.setProcessorEnabled(aprilTag, false);

            // drive your chosen sequence
            drivetrain.followTrajectorySequence(chosenSequence);

            // drive to the back drop
            drivetrain.followTrajectorySequence(goToBackdrop);

            //drive next sequence
                }


            }

            private void initAprilTag(){
                //Establish AprilTag detection
                // Create a new AprilTag Processor Builder object.
                aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(myAprilTagLibrary)
                        .setDrawTagID(true)
                        .setDrawTagOutline(true)
                        .setDrawAxes(true)
                        .setDrawCubeProjection(true)
                        .build();

                myVisionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .setCameraResolution(new Size(640, 480))
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .enableLiveView(true)
                        .setAutoStopLiveView(true)
                        .build();
            }


        private void telemetryAprilTag () {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));

                    markerLocation = detection.center.x;
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");


        }   // end method telemetryAprilTag()
}
