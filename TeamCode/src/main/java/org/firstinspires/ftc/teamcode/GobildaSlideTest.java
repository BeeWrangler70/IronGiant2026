/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Test Gobilda Slide")
public class GobildaSlideTest extends LinearOpMode
{

    private int targetPosition = 0;
    private DcMotorEx liftMotor1;

    @Override public void runOpMode()
    {
        // This setups the motor and RESETS THE ENCODER to 0
        liftMotor1 = setupDriveMotor( "liftMotor1",DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.addData("Instructions", "Press Gamepad 1 'Y' to increment position by 50 ticks.");
        telemetry.addData("Instructions", "Press Gamepad 1 'X' to increment position by 200 ticks.");
       telemetry.update();

        liftMotor1.setTargetPosition(targetPosition);
        // Switch the motor to RUN_TO_POSITION mode
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(1.0);

        waitForStart();

        while (opModeIsActive())
        {
            // Every time 'Y' is pressed, increment the target and run to it
            if (gamepad1.yWasPressed()) {
                // Increment the target position by 50 ticks
                targetPosition += 50;

                // Set the new target for the motor
                liftMotor1.setTargetPosition(targetPosition);

                // Switch the motor to RUN_TO_POSITION mode
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set power to the motor. It will run at this power until it reaches the target.
                liftMotor1.setPower(1.0);
            }

            if (gamepad1.aWasPressed()) {
                // Increment the target position by 50 ticks
                targetPosition -= 50;

                if (targetPosition < 0) {
                    targetPosition = 0;
                }

                // Set the new target for the motor
                liftMotor1.setTargetPosition(targetPosition);

                // Switch the motor to RUN_TO_POSITION mode
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set power to the motor. It will run at this power until it reaches the target.
                liftMotor1.setPower(1.0);
            }

            if (gamepad1.xWasPressed()) {
                // Increment the target position by 50 ticks
                targetPosition += 200;

                // Set the new target for the motor
                liftMotor1.setTargetPosition(targetPosition);

                // Switch the motor to RUN_TO_POSITION mode
                liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set power to the motor. It will run at this power until it reaches the target.
                liftMotor1.setPower(1.0);
            }

            // Add telemetry to see what the motor is doing
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", liftMotor1.getCurrentPosition());
            telemetry.addData("Motor is Busy", liftMotor1.isBusy());
            telemetry.update();
        }
    }

    private DcMotorEx setupDriveMotor(String deviceName, DcMotorEx.Direction direction) {
        DcMotorEx aMotor = hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        // This line ensures the lift starts at encoder position 0
        aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // We set this initially to prepare for RUN_TO_POSITION
        aMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        return aMotor;
    }


}
