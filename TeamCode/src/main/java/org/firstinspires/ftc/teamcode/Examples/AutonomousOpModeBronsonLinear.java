package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareAndFunctions;
import org.firstinspires.ftc.teamcode.Launcher;

/**
 * Autonomous OpMode (Linear OpMode) implementing Bronson's movement sequence
 * using HardwareAndFunctions / Proportional Controllers for precise movement.
 * <p>
 * Sequence:
 * <ul>
 *   <li>Turn on launcher flywheel (wait 1 sec)</li>
 *   <li>Turn on launcher feeder for 2 seconds, then stop</li>
 *   <li>Drive forward 12 inches</li>
 *   <li>Turn 90 degrees left (absolute heading)</li>
 *   <li>Drive forward 36 inches</li>
 *   <li>Strafe left 24 inches</li>
 *   <li>Drive forward 16 inches</li>
 *   <li>Drive backward 12 inches</li>
 *   <li>Strafe right 8 inches</li>
 *   <li>Drive forward 9 inches</li>
 *   <li>Stop</li>
 * </ul>
 */
@Disabled
@Autonomous(name = "Bronson Linear Autonomous", group = "Autonomous")
public class AutonomousOpModeBronsonLinear extends LinearOpMode {

    private HardwareAndFunctions robot;

    @Override
    public void runOpMode() {
        robot = new HardwareAndFunctions(this);
        telemetry.addData("Status", "Initializing Bronson Linear Autonomous...");
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

            // 4. Sequential Movement Sequence from Bronson Basic Autonomous
            robot.drive(12, 0.8, 0.25);         // Drive forward 12 inches (1 foot)
            robot.turnTo(90, 0.50, 0.25);         // Turn to absolute 90 degrees heading
            robot.drive(36, 0.8, 0.25);         // Drive forward 36 inches (3 feet)
            robot.strafe(24, 0.60, 0.15);       // Strafe left 24 inches (2 feet)
            robot.drive(16, 0.8, 0.25);         // Drive forward 16 inches
            robot.drive(-12, 0.8, 0.25);        // Drive backward 12 inches
            robot.strafe(8, 0.60, 0.15);        // Strafe right 8 inches (corrected from 96 inches)
            robot.drive(9, 0.8, 0.25);          // Drive forward 9 inches
        }

        // Clean up and stop all mechanisms at the end of autonomous
        robot.stopIntake();
        robot.stopLauncher();
        robot.stopRobot();
    }
}
