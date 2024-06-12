package org.firstinspires.ftc.teamcode.Auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous

public class RedShortVision extends LinearOpMode {

    VisionSubB visub = null;
    VisionPortal myVisionPortal;
    private TfodProcessor tfod;

    Trajectories trajectories;
    double markerLocation = 300;
    Constants constants = new Constants(this);
    private TrajectorySequence chosenSequence;
    private TrajectorySequence lastSequence;
    private TrajectorySequence lastEndSequence;
    private Pose2d backDropGoTO;
    private Pose2d newPose;
    private Pose2d lastPose;
    private Pose2d lastParkPose;


    final int e_tiltPickUp = 0; //The tilt position for picking up a pixel 320 for 5618 and 6494
    final int e_tiltStowed = -400; //The tilt position for moving across the field -30
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


        // Init the AprilTag processor and Vision Portal
//        initAprilTag();
        visub = new VisionSubB(hardwareMap, false);
        constants.init();
//        initTfod();


        //Instantiate the drive system
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap);
        //close claw and lift slide
        constants.claw.setPosition(constants.closed);
        sleep(1000);
        constants.slide_motor.setTargetPosition(slidePickup);
        //Provide the initial pose
        Pose2d blueLongStart = new Pose2d(-39, 60, Math.toRadians(270));
        Pose2d blueShortStart = new Pose2d(12, 60, Math.toRadians(270));
        Pose2d redLongStart = new Pose2d(-39, -60, Math.toRadians(90));
        Pose2d redShortStart = new Pose2d(12, -60, Math.toRadians(90));

        Pose2d startPose = redShortStart; //tell the robot where it starts

        //Occupy the initial pose
        drivetrain.setPoseEstimate(startPose);

        TrajectorySequence test = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -30, Math.toRadians(180)))
                .forward(5)
                .build();

        TrajectorySequence Left = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))
                .forward(5)
                .build();

        TrajectorySequence Center = drivetrain.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12, -29))
                .build();

        TrajectorySequence Right = drivetrain.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(24, -32, Math.toRadians(180)))
                .back(8)
                .build();


        //read Team Marker Position while waiting for start
        while (!isStarted() && !isStopRequested()) {
            if (visub.foundRED()) {
                telemetry.addData("RED Found", visub.getTFODX());
                telemetry.update();
                if (visub.getTFODX() > 350) {
                    newPose = Center.end(); // newPose is the end of the first sequence
                    chosenSequence = Center;
                    backDropGoTO = new Pose2d(54, -33, Math.toRadians(180));
                    telemetrySequence = 2;


                } else if (visub.getTFODX() < 300) {
                    newPose = Left.end();
                    chosenSequence = Left;
                    backDropGoTO = new Pose2d(54, -27, Math.toRadians(180));
                    telemetrySequence = 1;
                }
            }




             else {
                newPose = Right.end();
                chosenSequence = Right;
                backDropGoTO = new Pose2d(54, -40, Math.toRadians(180));
                telemetrySequence = 3;

            }
        }


        telemetry.addData("RED Found", visub.getTFODX());
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

        if (lastSequence == rightGoToBackdrop){
            lastPose = rightGoToBackdrop.end();
        }
        else if (lastSequence == leftGoToBackdrop) {
            lastPose = leftGoToBackdrop.end();
        }
        else {
            lastPose = centerGoToBackdrop.end();
        }


        TrajectorySequence goPark = drivetrain.trajectorySequenceBuilder(lastPose)
                .forward(5)
                .lineTo(new Vector2d(43,-60))
                .lineTo(new Vector2d(58,-60))

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

        TrajectorySequence endPart = drivetrain.trajectorySequenceBuilder(lastPose)
                .forward(9)
                .lineTo(new Vector2d(12,-60))
                .lineTo(new Vector2d(-36,-60))
                .lineTo(new Vector2d(-61,-36))

                .build();


        if (telemetrySequence == 3) {
            lastEndSequence = endPart;

        }
        else if (telemetrySequence == 2) {
            lastEndSequence = endPart;

        }
        else {
            lastEndSequence = endPart;

        }




//*********************************** START IS PRESSED ********************************************
        if (!isStopRequested()) {
            // Disable the AprilTag processor.
//            myVisionPortal.setProcessorEnabled(aprilTag, false);
            // Save more CPU re6sources when camera is no longer needed.
//            myVisionPortal.close();

            // drive your chosen sequence
            drivetrain.followTrajectorySequence(chosenSequence);
            //drop one pixel
            constants.claw.setPosition(constants.halfopen);
            sleep(500);
            constants.e_tilt.setTargetPosition(e_tiltStowed);
            sleep(500);

            // drive to the back drop
            constants.claw.setPosition(constants.closed);
            sleep(500);
            drivetrain.followTrajectorySequence(lastSequence);

            // score yellow pixel
            letsScore();
            //go pick up pixel
            drivetrain.followTrajectorySequence(lastEndSequence);
            //go park
//            drivetrain.followTrajectorySequence(goPark);
        }
    }

//    private void initAprilTag() {
//        //Establish AprilTag detection
//        // Create a new AprilTag Processor Builder object.
//        aprilTag = new AprilTagProcessor.Builder()
////                .setTagLibrary(myAprilTagLibrary)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .build();
//
//        myVisionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTag)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();
//    }


//        private void telemetryAprilTag () {
//
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//            telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//            // Step through the list of detections and display info for each one.
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                } else {
//                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//
//                    markerLocation = detection.center.x;
//                }
//            }   // end for() loop
//
//            // Add "key" information to telemetry
//            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//            telemetry.addLine("RBE = Range, Bearing & Elevation");
//        }// end method telemetryAprilTag()

    private void letsScore() {
        if (state == 0) {
            constants.e_tilt.setTargetPosition(e_tiltStowed);
            constants.claw.setPosition(constants.closed);
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
            constants.claw.setPosition(constants.halfopen);
            sleep(400);
            constants.claw.setPosition(open);
            sleep(100);
            state++;
        }
        //close the claw
        if (state == 5) {
            telemetry.addData("Debug", "Entering state 4");
            telemetry.update();
            constants.claw.setPosition(constants.closed);
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

    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
            myVisionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }   // end method initTfod()

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}
