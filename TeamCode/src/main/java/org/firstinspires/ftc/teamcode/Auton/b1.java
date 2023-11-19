package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous

public class b1 extends LinearOpMode {

    //    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    AprilTagProcessor aprilTag;
    double markerLocation;

    public void runOpMode() throws InterruptedException {

        //Establish AprilTag detection
        // Create a new AprilTag Processor Builder object.
//        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Optional: specify a custom Library of AprilTags.
//        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);   // The OpMode must have already created a Library.

// Optional: set other custom features of the AprilTag Processor (4 are shown here).
        aprilTag = new AprilTagProcessor.Builder()
//                .setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal myVisionPortal;

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

// Enable or disable the AprilTag processor. It is enabled by default, use this if you disable it and need to reenable it.
//        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        //Instantiate the drive system
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);

        //Provide the initial pose
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270)); //tell the robot where it starts

        //Occupy the initial pose
        drivetrain.setPoseEstimate(startPose);


//
//
//                .build();
        //read Team Marker Position while waiting for start

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

// Cycle through through the list and process each AprilTag.

//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//            markerLocation = detection.center.x;
//            telemetry.update();
//            sleep(100);
//        }
        waitForStart();


        if (opModeIsActive()) {
            while (opModeIsActive()) {


                telemetryAprilTag();
                telemetry.addData("marker center", markerLocation);
                // Push telemetry to the Driver Station.
                telemetry.update();//        while (!isStarted()) {
            if (markerLocation > 400) { //right
                //drive to center location

            } else if (markerLocation < 300) { //center
                //drive to right location

            } else { //left
                //drive to left location

            }
        }
//


            }
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
