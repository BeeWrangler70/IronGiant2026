package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous OpMode (Iterative OpMode) using a non-blocking state machine
 * and HardwareAndFunctions / Proportional Controllers for precise movement.
 * <p>
 * Sequence:
 * <ul>
 *   <li>Turn on launcher flywheel (wait 1 sec)</li>
 *   <li>Turn on launcher feeder for 2 seconds, then stop</li>
 *   <li>Drive forward 3 inches</li>
 *   <li>Strafe left 10 inches</li>
 *   <li>Turn 90 degrees left</li>
 *   <li>Turn on intake and drive forward 2 inches</li>
 *   <li>Turn 180 degrees to the right</li>
 *   <li>Stop</li>
 * </ul>
 */
@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class AutonomousOpMode extends OpMode {

    private HardwareAndFunctions robot;
    private Launcher launcher;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();

    private enum AutoState {
        SPIN_UP_LAUNCHER,
        WAIT_FOR_SPIN_UP,
        START_FEEDER,
        WAIT_FOR_FEED,
        STOP_LAUNCHER,
        DRIVE_FORWARD_3,
        STRAFE_LEFT_10,
        TURN_LEFT_90,
        TURN_ON_INTAKE_AND_DRIVE_2,
        TURN_RIGHT_180,
        DONE
    }

    private AutoState currentState = AutoState.SPIN_UP_LAUNCHER;

    @Override
    public void init() {
        robot = new HardwareAndFunctions(this);
        telemetry.addData("Status", "Initializing Iterative Autonomous...");
        telemetry.update();

        robot.initialize(true);
        launcher = robot.launcherSubsystem;

        telemetry.addData("Status", "Ready! Waiting for Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        stateTimer.reset();
        holdTimer.reset();
        currentState = AutoState.SPIN_UP_LAUNCHER;
    }

    @Override
    public void loop() {
        // 1. Always update sensors and subsystem states
        robot.readSensors();
        launcher.update();

        switch (currentState) {
            case SPIN_UP_LAUNCHER:
                robot.setLauncherVelocity(Launcher.LAUNCHER_TARGET_VELOCITY);
                stateTimer.reset();
                currentState = AutoState.WAIT_FOR_SPIN_UP;
                break;

            case WAIT_FOR_SPIN_UP:
                if (stateTimer.seconds() >= 1.0) {
                    currentState = AutoState.START_FEEDER;
                }
                break;

            case START_FEEDER:
                robot.turnOnFeeder();
                stateTimer.reset();
                currentState = AutoState.WAIT_FOR_FEED;
                break;

            case WAIT_FOR_FEED:  // probably takes 2 seconds to shoot 4 balls
                if (stateTimer.seconds() >= 2.0) {
                    currentState = AutoState.STOP_LAUNCHER;
                }
                break;

            case STOP_LAUNCHER:
                robot.stopFeeder();
                robot.stopLauncher();
                currentState = AutoState.DRIVE_FORWARD_3;
                break;

            case DRIVE_FORWARD_3:
                robot.drive(3,.8,.25);  // drive forward 3 inches with 80% power
                currentState = AutoState.STRAFE_LEFT_10;
                break;

            case STRAFE_LEFT_10:
                robot.strafe( 10, 0.60,0.15); // Strafe left 10 inches
                currentState = AutoState.TURN_LEFT_90;
                break;

            case TURN_LEFT_90:
                robot.turnTo(90, 0.50, 0.25);  // Turn 90 degrees left
                currentState = AutoState.TURN_ON_INTAKE_AND_DRIVE_2;
                break;

            case TURN_ON_INTAKE_AND_DRIVE_2:
                robot.turnOnIntake();
                robot.drive(2,.8,.25);  // drive forward 2 inches with 80% power
                currentState = AutoState.TURN_RIGHT_180;
                break;

            case TURN_RIGHT_180:
                robot.turnTo(270, 0.50, 0.25);  // Turn 180 degrees right
                currentState = AutoState.DONE;
                break;

            case DONE:
                robot.stopIntake();
                robot.stopLauncher();
                robot.stopRobot();
                break;
        }

        // Telemetry status update
        telemetry.addData("Current State", currentState);
        telemetry.addData("Position (X, Y)", "%.2f, %.2f", robot.driveDistance, robot.strafeDistance);
        telemetry.addData("Heading", "%.1f", robot.heading);
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopIntake();
        robot.stopLauncher();
        robot.stopRobot();
    }
}
