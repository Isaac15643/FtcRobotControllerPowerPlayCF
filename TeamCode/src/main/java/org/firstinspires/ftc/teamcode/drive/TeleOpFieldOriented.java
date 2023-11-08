/* Driving with mech wheels
 *
 */

package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*********************************************/

@TeleOp(name="TeleOpFieldOriented", group="Linear Opmode")
//@Disabled

public class TeleOpFieldOriented extends LinearOpMode {

    Constants constants = new Constants(this);
    Commands commands;

    // State used for updating telemetry
    Orientation angles;

    private double left_front_power;
    private double right_front_power;
    private double left_rear_power;
    private double right_rear_power;

    public double headingOffset = 0;
    public double robotHeading = 0;
    public double headingError = 0;

    private boolean dPadUpIsPressed = false;
    private boolean dPadDownIsPressed = false;
    private boolean dPadLeftIsPressed = false;
    private boolean dPadRightIsPressed = false;
    private int scoreY = 1; //vertical scoring position
    private int scoreX = 1; //horizontal scoring position




    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()

//    private double targetHeading = 0;
//    private double driveSpeed = 0;
//    private double turnSpeed = 0;
//    private double leftSpeed = 0;
//    private double rightSpeed = 0;
//    private final int leftFrontTarget = 0;
//    private final int leftRearTarget = 0;
//    private final int rightFrontTarget = 0;
//    private final int rightRearTarget = 0;

    @Override
    public void runOpMode() {

        constants.init();
        telemetry.update();
        constants.DRIVE_SPEED = 0.5;
        constants.TURN_SPEED = 0.50;



        double driveTurn;
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot = 0;
        double gamepadRadians = 0;
        double robotRadians = 0;
        double correctedRobotRadians = 0;
        double movementRadians = 0;
        double gamepadXControl = 0;
        double gamepadYControl = 0;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)


            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
            angles = constants.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            angles = constants.imu.getRobotYawPitchRollAngles();
            float gyro_degrees = (angles.firstAngle) - (float) headingOffset;
            telemetry.addData("Score X", scoreX);
            telemetry.addData("Score Y", scoreY);
            telemetry.addData("Yaw", ("%.3f"), gyro_degrees);
            telemetry.addData("e_tilt", constants.e_tilt.getCurrentPosition());
            telemetry.addData("slide_motor", constants.slide_motor.getCurrentPosition());
            telemetry.addData("Left X", ("%.3f"), gamepad1.left_stick_x);
            telemetry.addData("Right X", ("%.3f"), gamepad2.right_stick_x);
            telemetry.addData("Right Y", ("%.3f"), gamepad2.right_stick_y);
            telemetry.addData("gamepadHypot", ("%.3f"), gamepadHypot);
            telemetry.addData("gamepadDegree", ("%.3f"), gamepadRadians);
            telemetry.addData("movementDegree", ("%.3f"), movementRadians);
            telemetry.addData("gamepadXControl", ("%.3f"), gamepadXControl);
            telemetry.addData("gamepadYControl", ("%.3f"), gamepadYControl);
            telemetry.addData("RF POWER", ("%.3f"), right_front_power);
            telemetry.addData("RR POWER", ("%.3f"), right_rear_power);
            telemetry.addData("LF POWER", ("%.3f"), left_front_power);
            telemetry.addData("LR POWER", ("%.3f"), left_rear_power);


            driveTurn = -gamepad1.left_stick_x;
            gamepadXCoordinate = gamepad1.right_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = -gamepad1.right_stick_y; //this simply gives our y value relative to the driver
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);

            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)

            gamepadRadians = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);// - Math.PI/2; //the inverse tangent of opposite/adjacent gives us our gamepad degree

            robotRadians = (gyro_degrees * Math.PI / 180); //gives us the angle our robot is at, in radians

            movementRadians = gamepadRadians - robotRadians; //adjust the angle we need to move at by finding needed
            // movement degree based on gamepad and robot angles
            gamepadXControl = Math.cos(movementRadians) * gamepadHypot;
            //by finding the adjacent side, we can get our needed x value to power our motors
            gamepadYControl = Math.sin(movementRadians) * gamepadHypot;
            //by finding the opposite side, we can get our needed y value to power our motors

            //by multiplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will
            // not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
            right_front_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            right_rear_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            left_front_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            left_rear_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            constants.rightFront.setPower(right_front_power * constants.DRIVE_SPEED);
            constants.leftFront.setPower(left_front_power * constants.DRIVE_SPEED);
            constants.rightRear.setPower(right_rear_power * constants.DRIVE_SPEED);
            constants.leftRear.setPower(left_rear_power * constants.DRIVE_SPEED);

            //Declare other button functions here
            //*****************************     Gamepad 1     **************************************
            //**************   ROBOT SPEED   **************
            if (gamepad1.left_bumper) { // slow down for precision
                constants.DRIVE_SPEED = 0.25;
            } else {
                constants.DRIVE_SPEED = 1.0;
            }

            //Spin 180 degrees
            if (gamepad1.left_bumper) {
//
            }

            //Reset Heading
            while (gamepad1.right_bumper) {
                constants.resetHeading();
            }

            //*****************************     Gamepad 2     **************************************
            //
            if (gamepad2.b) {
                constants.claw.setPosition(.5);

            }



            int e_tiltPickUp = 270; //The tilt position for picking up a pixel
            int e_tiltStowed = 0; //The tilt position for moving across the field
            double p_tiltPickup = 0; //The tilt position of the claw mechanism for picking up a pixel
            double p_tiltScore = 0.75; //The tilt position of the claw mechanism for scoring a pixel
            int slidePickup = -160;
            int slideLow = -1300;
            int slideMed = -1900;
            int slideHigh = -2600;
            int slideTop = -3100;
            int e_tiltPickup = 270;

            if (gamepad2.a) {
                // set slide extension to pickup position
                constants.e_tilt.setTargetPosition(slidePickup);
                constants.e_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                constants.e_tilt.setPower(0.5);

                // set elevator tilt and pixel tilt to pickup position (90 deg vertical)
                constants.e_tilt.setTargetPosition((e_tiltPickUp));
                constants.e_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                constants.e_tilt.setPower(0.5);
                constants.p_tilt.setPosition(0);
                constants.claw.setPosition(1);

//drop elevator down to get pixel
                constants.slide_motor.setTargetPosition(0);

                //Close the claw
                constants.claw.setPosition(0);

                // set elevator tilt position to stowed position
                constants.e_tilt.setTargetPosition(e_tiltStowed);            } else {}

           //Pickup a pixel
            if (gamepad2.y) {
//            constants.getPixel();
                constants.clawCollect();
            } else {}


            if (gamepad2.right_stick_y < -0.2 || gamepad2.right_stick_y > 0.2) { //move slide manually
                constants.slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                constants.slide_motor.setPower(gamepad2.right_stick_y);
            } else {
//                constants.slide_motor.setPower(0);
            }

            if (gamepad2.left_stick_y < -0.2 || gamepad2.left_stick_y > 0.2) { //tilt elevator manually
                constants.e_tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                constants.e_tilt.setPower(gamepad2.left_stick_y * 0.5);
            } else {
//                constants.e_tilt.setPower(0);
            }

            //set scoring position
            if (gamepad2.dpad_up) {
                if (!dPadUpIsPressed) {
                    scoreY++;
                    if (scoreY > 4) {
                        scoreY = 4;
                    }
                    dPadUpIsPressed = true;
                }
            } else {
                dPadUpIsPressed = false;
            }

            if (gamepad2.dpad_down) {
                if (!dPadDownIsPressed) {
                    scoreY--;
                    if (scoreY < 1) {
                        scoreY = 1;
                    }
                    dPadDownIsPressed = true;
                }
            } else {
                dPadDownIsPressed = false;
            }

            if (gamepad2.dpad_left) {
                if (!dPadLeftIsPressed) {
                    scoreX--;
                    if (scoreX < 1) {
                        scoreX = 1;
                    }
                    dPadLeftIsPressed = true;
                }
            } else {
                dPadLeftIsPressed = false;
            }

            if (gamepad2.dpad_right) {
                if (!dPadRightIsPressed) {
                    scoreX++;
                    if (scoreX > 3) {
                        scoreX = 3;
                    }
                    dPadRightIsPressed = true;
                }
            } else {
                dPadRightIsPressed = false;
            }


            if (gamepad2.right_bumper) {} else {}

            if (gamepad2.x) {
                constants.claw.setPosition(0);

            } else {}
            
            telemetry.update();

        } //End of while op mode is active

    }//End of run OP Mode

    public void runToPosition(){
        constants.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        constants.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        constants.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        constants.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        constants.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        constants.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        constants.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        constants.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder(){
        constants.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        constants.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        constants.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        constants.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


//    public double getRawHeading() {
//        Orientation angles   = constants.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle;
//    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading () {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = angles.firstAngle;
        robotHeading = 0;
    }

//        //Constants and functions for adding automatic steering controls
//        static final double COUNTS_PER_MOTOR_REV = 28.0;   // Rev Ultraplanetary HD Hex motor: 28.0
//        static final double DRIVE_GEAR_REDUCTION = 20.0;     // External Gear Ratio
//        static final double WHEEL_DIAMETER_INCHES = 3.78;     // 96mm Mech Wheels, For figuring circumference
//        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                (WHEEL_DIAMETER_INCHES * 3.1415);
//
//        // These constants define the desired driving/control characteristics
//        // They can/should be tweaked to suit the specific robot drive train.
//        static double DRIVE_SPEED = .75;     // Max driving speed for better distance accuracy.
//        static final double TURN_SPEED = 1.0;     // Max Turn speed to limit turn rate
//        static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
//        // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
//        // Define the Proportional control coefficient (or GAIN) for "heading control".
//        // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
//        // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
//        // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
//        static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
//        static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
//
//        public void turnToHeading ( double maxTurnSpeed, double heading){
//
//            // Run getSteeringCorrection() once to pre-calculate the current error
//            getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//            // keep looping while we are still active, and not on heading.
//            while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
//
//                // Determine required steering to keep on heading
//                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//                // Clip the speed to the maximum permitted value.
//                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//
//                // Pivot in place by applying the turning correction
//                moveRobot(0, turnSpeed);
//
//            }
//            moveRobot(0, 0);
//        }


//        public double getSteeringCorrection ( double desiredHeading, double proportionalGain){
//            targetHeading = desiredHeading;  // Save for telemetry
//
//            // Get the robot heading by applying an offset to the IMU heading
//            robotHeading = angles.firstAngle - headingOffset;
//
//            // Determine the heading current error
//            headingError = targetHeading - robotHeading;
//
//            // Normalize the error to be within +/- 180 degrees
//            while (headingError > 180) headingError -= 360;
//            while (headingError <= -180) headingError += 360;
//
//            // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
//            return Range.clip(headingError * proportionalGain, -1, 1);
//        }
//
//        public void moveRobot ( double drive, double turn){
//            driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
//            turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.
//
//            leftSpeed = drive - turn;
//            rightSpeed = drive + turn;
//
//            // Scale speeds down if either one exceeds +/- 1.0;
//            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//            if (max > 1.0) {
//                leftSpeed /= max;
//                rightSpeed /= max;
//            }
//
//            constants.leftFront.setPower(leftSpeed);
//            constants.leftRear.setPower(leftSpeed);
//            constants.rightFront.setPower(rightSpeed);
//            constants.rightRear.setPower(rightSpeed);
//        }
}
