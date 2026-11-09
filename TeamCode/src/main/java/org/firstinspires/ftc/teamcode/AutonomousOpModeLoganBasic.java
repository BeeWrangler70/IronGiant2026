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
@Autonomous(name = "Logan Basic Autonomous", group = "Autonomous")
public class AutonomousOpModeLoganBasic extends OpMode {

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
        DRIVE_FORWARD_1,
        DONE,
        STRAFE_LEFT_2,
        DRIVE_FORWARD_1FT_4INCHES,
        DRIVE_BACK_1,
        STRAFE_RIGHT_8,
        STRAFE_LEFT_3,
        DRIVE_FORWARD_7_5,
        TURN_90_LEFT,
        DRIVE_FORWARD_24,
        DRIVE_FORWARD_1FT_6INCHES,
        STRAFE_LEFT_8,
        DRIVE_FORWARD_9,
        STRAFE_LEFT_7


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
                if (stateTimer.seconds() >=2.5) //perfect amount of time for 1st shot
                {
                    currentState = AutoState.START_FEEDER;
                }
                break;

            case START_FEEDER:
                robot.turnOnFeeder();
                stateTimer.reset();
                currentState = AutoState.WAIT_FOR_FEED;
                break;

            case WAIT_FOR_FEED:  // probably takes 2 seconds to shoot 4 balls
                if (stateTimer.seconds() >= 5.0) {
                    currentState = AutoState.STOP_LAUNCHER;
                }
                break;

            case STOP_LAUNCHER:
                robot.stopFeeder();
                robot.stopLauncher();
                currentState = AutoState.TURN_90_LEFT;
                break;

            case TURN_90_LEFT:
                robot.turnTo(-90, 0.8, 0.25);
                robot.toggleIntake();   //WE TURNNED ON THE INTAKE
                currentState = AutoState.STRAFE_LEFT_2;
                break;
            case STRAFE_LEFT_2:
                robot.strafe(2,0.90, 0.15);
                currentState = AutoState.DRIVE_FORWARD_24;
                break;

            case DRIVE_FORWARD_24:
                robot.drive(24,0.9,0.25);
                currentState = AutoState.DRIVE_FORWARD_1FT_6INCHES;;
                break;

            case DRIVE_FORWARD_1FT_6INCHES:_6:
                robot.drive(18,.2,.25);  // drive forward 3 inches with 80% power
                currentState = AutoState.DRIVE_BACK_1;
                break;

            case DRIVE_BACK_1:
                robot.drive( -12, 0.90,0.15); // Strafe left 10 inches
                currentState = AutoState.STRAFE_LEFT_7;
                break;
                case STRAFE_LEFT_7:
                    robot.strafe( -84, 0.90,0.15); // Strafe left 10 inches
                    currentState = AutoState.DRIVE_FORWARD_9;
                    break;
                    case DRIVE_FORWARD_9:
                        robot.drive( 9, 0.90,0.15); // Strafe left 10 inches
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
