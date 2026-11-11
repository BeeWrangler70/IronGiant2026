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
 *   <li>Turn on launcher flywheel (wait 1.5 sec)</li>
 *   <li>Shoot 4 balls using feedQuarterTurn with 0.5 second waits between each shot</li>
 *   <li>Stop launcher</li>
 *   <li>Drive and strafe movements</li>
 * </ul>
 */
@Autonomous(name = "Autonomous 1", group = "Autonomous")
public class AutonomousOpModeBronsonBasic extends OpMode {

    private HardwareAndFunctions robot;
    private Launcher launcher;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime holdTimer = new ElapsedTime();
    private int ballsShot = 0;

    private final double WAIT_FOR_SPIN_UP_TIME = 1.5;
    private final int BALLS_TO_SHOOT = 6;

    private final double WAIT_BETWEEN_SHOTS_TIME = 0.75;

    private enum AutoState {
        SPIN_UP_LAUNCHER,
        WAIT_FOR_SPIN_UP,
        START_FEEDER,
        SHOOT_BALL,
        WAIT_BETWEEN_SHOTS,
        STOP_LAUNCHER,
        DRIVE_FORWARD_1,
        DONE,
        STRAFE_LEFT_2,
        DRIVE_FORWARD_1FT_4INCHES,
        DRIVE_BACK_1,
        STRAFE_RIGHT_8,
        STRAFE_LEFT_3,
        DRIVE_FORWARD_7_5
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
        ballsShot = 0;
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
                if (stateTimer.seconds() >= WAIT_FOR_SPIN_UP_TIME) {
                    currentState = AutoState.START_FEEDER;
                }
                break;

            case START_FEEDER:
                ballsShot = 0;
                currentState = AutoState.SHOOT_BALL;
                break;

            case SHOOT_BALL:
                if (ballsShot < BALLS_TO_SHOOT) {
                    robot.feedQuarterTurn();
                    ballsShot++;
                    stateTimer.reset();
                    currentState = AutoState.WAIT_BETWEEN_SHOTS;
                } else {
                    currentState = AutoState.STOP_LAUNCHER;
                }
                break;

            case WAIT_BETWEEN_SHOTS:
                if (stateTimer.seconds() >= WAIT_BETWEEN_SHOTS_TIME) {
                    currentState = AutoState.SHOOT_BALL;
                }
                break;

            case STOP_LAUNCHER:
                robot.stopFeeder();
                robot.stopLauncher();
                currentState = AutoState.DRIVE_FORWARD_1;
                break;

            case DRIVE_FORWARD_1:
                robot.drive(-12,.8,.25);  // drive forward 3 inches with 80% power
                currentState = AutoState.STRAFE_LEFT_2;
                break;

            case STRAFE_LEFT_2:
                robot.strafe( -24, 0.60,0.15); // Strafe left 10 inches
                currentState = AutoState.DRIVE_FORWARD_7_5;
                break;

            case DRIVE_FORWARD_7_5:
                robot.drive(-72,.8,.25);  // drive forward 3 inches with 80% power
                currentState = AutoState.STRAFE_LEFT_3;
                break;

            case STRAFE_LEFT_3:
                robot.strafe( -15, 0.60,0.15); // Strafe left 10 inches
                currentState = AutoState.DONE;
                break;

            case DONE:
                robot.stopIntake();
                robot.stopLauncher();
                robot.stopRobot();
                break;
        }

        // Telemetry status update
        robot.sendTelemetry(currentState, ballsShot);
    }

    @Override
    public void stop() {
        robot.stopIntake();
        robot.stopLauncher();
        robot.stopRobot();
    }
}
