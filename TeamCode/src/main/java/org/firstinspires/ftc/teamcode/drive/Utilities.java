package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Utilities", group="Linear Opmode")
public class Utilities extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Constants constants = new Constants(this);
        constants.init();
        telemetry.update();
        int hangerTarget = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addLine("Gamepad 2 Right Bumper to lower hook");
            telemetry.addLine("Gamepad 2 Left Bumper to raise hook");
            telemetry.addLine("Gamepad 2 Right Trigger to reset Encoders");

            telemetry.addData("Hanger Encoder", hangerTarget);



            hangerTarget = constants.hanger.getCurrentPosition();
            if (gamepad2.right_bumper) {
                constants.hanger.setTargetPosition(hangerTarget + 100);
                constants.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                constants.hanger.setPower(1);
            }

            if (gamepad2.left_bumper) {
                constants.hanger.setTargetPosition(hangerTarget - 100);
                constants.hanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                constants.hanger.setPower(1);
            }

            if (gamepad2. right_trigger > 0.5) {
                constants.e_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                constants.slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                constants.hanger.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.update();
        }

    }
}
