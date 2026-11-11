package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareAndFunctions;
import org.firstinspires.ftc.teamcode.Launcher;

/**
 * Autonomous OpMode (Linear OpMode) using HardwareAndFunctions / Proportional Controllers
 * for precise movement.
 * <p>
 * Sequence:
 * <ul>
 *   <li>Turn on launcher flywheel (wait 1 sec)</li>
 *   <li>Turn on launcher feeder for 2 seconds, then stop</li>
 *   <li>Drive forward 3 inches</li>
 *   <li>Strafe left 10 inches</li>
 *   <li>Turn 90 degrees left</li>
 *   <li>Turn on intake and drive forward 2 inches</li>
 *   <li>Turn to 270 degrees absolute heading</li>
 *   <li>Stop</li>
 * </ul>
 */
@Disabled
@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class AutonomousExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        HardwareAndFunctions robot = new HardwareAndFunctions(this);
        telemetry.addData("Status", "Initializing Linear Autonomous...");
        telemetry.update();

        robot.initialize(true);

        telemetry.addData("Status", "Ready! Waiting for Start.");
        telemetry.update();

        // Wait for Start button on Driver Station
        waitForStart();

        if (opModeIsActive()) {
            // 1. Spin up launcher flywheel
            robot.setLauncherVelocity(Launcher.LAUNCHER_TARGET_VELOCITY);
            sleep(1000); // Wait 1 second for flywheel to stabilize speed

            // 2. Start feeder to shoot
            robot.turnOnFeeder();
            sleep(2000); // Take ~2 seconds to feed/shoot balls

            // 3. Stop launcher and feeder
            robot.stopFeeder();
            robot.stopLauncher();

            // 4. Autonomous Movement Sequence
            robot.drive(3, 0.8, 0.25);          // Drive forward 3 inches with 80% power
            robot.strafe(10, 0.60, 0.15);       // Strafe left 10 inches
            robot.turnTo(90, 0.50, 0.25);       // Turn to absolute 90 degrees left
            robot.turnOnIntake();               // Turn on intake
            robot.drive(2, 0.8, 0.25);          // Drive forward 2 inches with 80% power
            robot.turnTo(270, 0.50, 0.25);      // Turn to absolute 270 degrees
        }

        // Clean up and stop all mechanisms at the end of autonomous
        robot.stopIntake();
        robot.stopLauncher();
        robot.stopRobot();
    }
}
