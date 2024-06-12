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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*********************************************/

@TeleOp(name="TeleOpFieldOriented", group="Linear Opmode")
//@Disabled

public class TeleOpFieldOriented extends LinearOpMode {

    Constants constants = new Constants(this);
    Commands commands;
    //initalize touch sensors
    TouchSensor e_tilt_up;
    // Touch sensor for lower limit of elevator CH 2-3
    TouchSensor e_stop;
    // Touch sensor for tilt upper limit of elevator CH 4-5
    TouchSensor e_tilt_down;

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
    int state = 0;
    int slideOldPosition = 20;
    int lastSlidePosition = 0;
    int slideNewPosition = 0;
    int slideTarget = 0;
    int e_tiltTarget = 0;
    int hangerTarget = 0;

    int slideCounter = 0;
    int e_tiltCounter = 0;
    boolean zeroingTilt = false;
    boolean uprightingTilt = false;

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
        e_tilt_up = hardwareMap.get(TouchSensor.class, "e_tilt_up");
        // Touch sensor for lower limit of elevator CH 2-3
        e_stop = hardwareMap.get(TouchSensor.class, "e_stop");
        // Touch sensor for tilt upper limit of elevator CH 4-5
        e_tilt_down = hardwareMap.get(TouchSensor.class, "e_tilt_down");
        constants.hanger.setPower(1.0);
        constants.slide_motor.setPower(1.0);
        initMechs();
        constants.e_tilt.setPower(0);
        constants.e_tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        constants.DRIVE_SPEED = 0.5;
        constants.TURN_SPEED = 0.50;

        YawPitchRollAngles orientation;

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

        boolean pickingUp = false;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)

            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
//            angles = constants.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            orientation = constants.imu.getRobotYawPitchRollAngles();
//            float gyro_degrees = (angles.firstAngle) - (float) headingOffset;
            double gyro_degrees = (orientation.getYaw(AngleUnit.DEGREES)) - (float) headingOffset;

//            telemetry.addData("Score X", scoreX);
//            telemetry.addData("Score Y", scoreY);
            telemetry.addData("Yaw", ("%.3f"), gyro_degrees);
            telemetry.addData("e_tilt current", constants.e_tilt.getCurrentPosition());
            telemetry.addData("e_tilt target", constants.e_tilt.getTargetPosition());
            telemetry.addData("hanger", constants.hanger.getCurrentPosition());
            telemetry.addData("slide target", constants.slide_motor.getTargetPosition());
            telemetry.addData("slide_motor", constants.slide_motor.getCurrentPosition());
            telemetry.addData("slide old position", slideOldPosition);
            telemetry.addData("slide last position", lastSlidePosition);
            telemetry.addData("state", state);
            telemetry.addData("e_tilt_up", e_tilt_up.isPressed());
            telemetry.addData("e_tilt_down", e_tilt_down.isPressed());
            telemetry.addData("e_stop", e_stop.isPressed());
            telemetry.addData("gamepad2.right_stick_y", gamepad2.right_stick_y);
            telemetry.addData("gamepad2.left_stick_y", gamepad2.left_stick_y);


            driveTurn = -gamepad1.left_stick_x;
            gamepadXCoordinate = -gamepad1.right_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = gamepad1.right_stick_y; //this simply gives our y value relative to the driver
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

            final int e_tiltPickUp = 0; //The tilt position for picking up a pixel 320 for 5618 and 6494
            final int e_tiltStowed = -475; //The tilt position for moving across the field -30

            final int slidePickup = -250;
            final int slideLow = -1300;
            final int slideMed = -1900;
            final int slideHigh = -2600;
            final int slideTop = -3100;
            boolean tryingToScore = false;

            //Declare other button functions here
            //*****************************     Gamepad 1     **************************************
            //**************   ROBOT SPEED   **************
            if (gamepad1.left_bumper) { // slow down for precision
                constants.DRIVE_SPEED = 0.5;
            } else if (gamepad1.right_trigger > 0.2) {
                constants.DRIVE_SPEED = 0.25;
            } else {
                constants.DRIVE_SPEED = 1.0;
            }
            //Reset Heading
            if (gamepad1.right_bumper) {
//                constants.resetHeading();
//                headingOffset = constants.imu.resetYaw();
//                robotHeading = 0;
                constants.imu.resetYaw();
            }

            if (gamepad1.x) {
                constants.hanger.setPower(0);
            }

            if (gamepad1.a) {
                constants.drone.setPosition(1);
            }

            if (gamepad1.b) {
                constants.drone.setPosition(0);
            }

            //*****************************     Gamepad 2     **************************************
            //close claw when slide is up
            if (constants.slide_motor.getCurrentPosition() < -550 && !tryingToScore) {
                constants.claw.setPosition(constants.closed);
            }
            //
//GAMEPAD2.A collect pixels
            // Pick up two pixels with one button from e_tilt upright
            if (gamepad2.a) {
                // open claw (wait a bit if its not already open)
                if (state == 0) {
                    if (constants.claw.getPosition() < 1 || constants.p_tilt.getPosition() > 0) {
                        constants.claw.setPosition(constants.open);
                        constants.p_tilt.setPosition(0);
                        sleep(250);
                    } else {
                        constants.claw.setPosition(constants.open);
                        constants.p_tilt.setPosition(0);
                    }
                    state++;
                }
                // Put slide down
                if (state == 1) {
                    uprightingTilt = true;
                    constants.slide_motor.setTargetPosition(0);
                    constants.slide_motor.setPower(0.3); //reduce power to prevent overdrive
                    state++;
                }
                //look for the slide to stop moving (landed on pixels)
                if (state == 2) {
                    sleep(200); //wait a bit for the values to change
                    //evaluate the difference between the current value and old value. If it's still moving it will
                    //be more than 5 ticks in 100 ms
                    while (opModeIsActive() && Math.abs(constants.slide_motor.getCurrentPosition() - slideOldPosition) > 5) {
                        //update the variable inside the while loop
                        telemetry.addData("diff", Math.abs(constants.slide_motor.getCurrentPosition() - slideOldPosition));
                        telemetry.update();
                        slideOldPosition = constants.slide_motor.getCurrentPosition();
                        sleep(300);
                    }
                    state++;
                    lastSlidePosition = constants.slide_motor.getCurrentPosition();
                }
                if (state == 3) {
                    constants.slide_motor.setPower(1.0); //reduce power to prevent overdrive
                    constants.slide_motor.setTargetPosition(lastSlidePosition - 10);//pop the slide up a bit
                    while (opModeIsActive() && Math.abs(constants.slide_motor.getCurrentPosition() - lastSlidePosition) > 5) {
                        //update the variable inside the while loop
                        telemetry.addData("diff", Math.abs(constants.slide_motor.getCurrentPosition() - slideOldPosition));
                        telemetry.update();
                        slideOldPosition = constants.slide_motor.getCurrentPosition();
                        sleep(500);
                    }
                    constants.claw.setPosition(constants.closed); //close the claw
                    state++;
                }

                // wait until claw is closed
                if (state == 4) {
                    sleep(500);
                    state++;
                }
                // pick up slide just a little bit more to clear the stack
                if (state == 5) {
                    constants.slide_motor.setTargetPosition(slidePickup);
                    state++;
                }
                //tilt the slide to stowed position
                if (state == 6) {
//                    constants.e_tilt.setTargetPosition(-250);
                    state = 0;
                    slideOldPosition = 20;
                    zeroingTilt = true;
                    uprightingTilt = false;
                }
                telemetry.update();
            }// done

// GAMEPAD2.Y Score Pixels
            if (gamepad2.y) {
                // run slide out to clearance position 
                tryingToScore = true;
                if (state == 0) {
                    zeroingTilt = true;
                    uprightingTilt = false;
                    constants.claw.setPosition(constants.closed);
                    state++;
                }
                if (state == 1) {
                    constants.slide_motor.setTargetPosition(slideMed);
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
                    constants.claw.setPosition(constants.open);
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
                    state++;
                }
                if (state == 8) {
                    telemetry.addData("Debug", "Entering state 7");
                    telemetry.update();
                    constants.claw.setPosition(constants.open);
                    state = 0;

                    tryingToScore = false;
                }
            }
//GAMEPAD2 X upright the slide and open the claw

            if (gamepad2.x) {
                uprightingTilt = true;
                zeroingTilt = false;
                constants.claw.setPosition(constants.open);
            }

            if (uprightingTilt) {
                if (!e_tilt_up.isPressed()) {
                    constants.e_tilt.setPower(0.4);
                } else {
                    constants.e_tilt.setPower(0);
                }
            }
            // Zero the tilt - stow slide
            if (gamepad2.b) {
                zeroingTilt = true;
                uprightingTilt = false;
            }

            if (zeroingTilt) {
                if (!e_tilt_down.isPressed()) {
                    constants.e_tilt.setPower(-0.4);
                } else {
                    constants.e_tilt.setPower(0);
                }

                if (gamepad2.right_stick_button) {
                    constants.claw.setPosition(constants.closed);
                }

                // Zero the tilt
                if (gamepad2.left_trigger > 0.5) {
                    while (opModeIsActive() && !e_tilt_down.isPressed()) {
                        constants.e_tilt.setTargetPosition(e_tiltTarget + 100);
                        constants.e_tilt.setPower(0.5);
                    }
                    constants.e_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    constants.e_tilt.setTargetPosition(0);
                    constants.e_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


                slideTarget = constants.slide_motor.getCurrentPosition();
                e_tiltTarget = constants.e_tilt.getCurrentPosition();
                hangerTarget = constants.hanger.getCurrentPosition();

                if (gamepad2.right_stick_y < -0.2) { //move slide up manually
                    if (slideTarget > slideMed) {
                        slideTarget -= 150;
                    } else {
                        slideTarget = slideMed;
                    }
                    constants.slide_motor.setTargetPosition(slideTarget);

                } else if (gamepad2.right_stick_y > 0.2) {
                    if (slideTarget < -10) {
                        slideTarget += 150;
                    } else {
                        slideTarget = -10;
                    }
                    constants.slide_motor.setTargetPosition(slideTarget);
                }

                if (gamepad2.left_stick_y > 0.2) { //tilt slide manually
                    if (e_tiltTarget >= e_tiltStowed) {
                        e_tiltTarget -= 20;
                    } else {
                        e_tiltTarget = e_tiltStowed;
                    }
                    constants.e_tilt.setTargetPosition(e_tiltTarget);

                } else if (gamepad2.left_stick_y < -0.2) {
                    if (e_tiltTarget <= e_tiltPickUp) {
                        e_tiltTarget += 20;
                    } else {
                        e_tiltTarget = e_tiltPickUp;
                    }
                    constants.e_tilt.setTargetPosition(e_tiltTarget);
                }

//            set scoring position
                if (gamepad2.dpad_up) {
                    constants.p_tilt.setPosition(1);
                }
                if (gamepad2.dpad_down) {
                    constants.p_tilt.setPosition(0);
                }

//
//            if (gamepad2.dpad_down) {
//                if (!dPadDownIsPressed) {
//                    scoreY--;
//                    if (scoreY < 1) {
//                        scoreY = 1;
//                    }
//                    dPadDownIsPressed = true;
//                }
//            } else {
//                dPadDownIsPressed = false;
//            }
//
//            if (gamepad2.dpad_left) {
//                if (!dPadLeftIsPressed) {
//                    scoreX--;
//                    if (scoreX < 1) {
//                        scoreX = 1;
//                    }
//                    dPadLeftIsPressed = true;
//                }
//            } else {
//                dPadLeftIsPressed = false;
//            }
//
//            if (gamepad2.dpad_right) {
//                if (!dPadRightIsPressed) {
//                    scoreX++;
//                    if (scoreX > 3) {
//                        scoreX = 3;
//                    }
//                    dPadRightIsPressed = true;
//                }
//            } else {
//                dPadRightIsPressed = false;
//            }

                if (gamepad2.right_bumper) {
                    constants.hanger.setTargetPosition(-30000);
                    constants.hanger.setPower(1);
                }

                if (gamepad2.left_bumper) {
                    constants.hanger.setTargetPosition(-9000);
                    constants.hanger.setPower(1);
                }
                telemetry.update();

            } //End of while op mode is active
            telemetry.update();
        }//End of run OP Mode
    }

    private void initMechs() {
        constants.p_tilt.setPosition(0);
        constants.claw.setPosition(1);
        constants.e_tilt.setTargetPosition(0);
        constants.hanger.setTargetPosition(0);
        constants.slide_motor.setTargetPosition(-100);
        constants.e_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        constants.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        constants.slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        constants.slide_motor.setPower(0.5);


        while (!e_tilt_up.isPressed()) {
            e_tiltCounter = constants.e_tilt.getCurrentPosition() + 10;
            constants.e_tilt.setTargetPosition(e_tiltCounter);
            sleep(20);
        }
        constants.e_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        constants.e_tilt.setPower(0.5);
        constants.e_tilt.setTargetPosition(0);
        constants.e_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (!e_stop.isPressed()) {
            slideCounter = constants.slide_motor.getCurrentPosition() + 50;
            constants.slide_motor.setTargetPosition(slideCounter);
            sleep(20);
        }
        constants.slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        constants.slide_motor.setPower(1.0);
        constants.slide_motor.setTargetPosition(-250);
        constants.slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData(">", "Mechs Initialized");
        telemetry.update();

    }
}




