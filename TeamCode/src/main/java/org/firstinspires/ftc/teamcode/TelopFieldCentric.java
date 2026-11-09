package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Main Field-Centric TeleOp OpMode (Iterative OpMode) demonstrating driver control
 * with GamepadController, field-centric drive, and mechanism control with the Launcher and Intake subsystems.
 * <p>
 * Driver Controls (Gamepad 1):
 * <ul>
 *   <li>Left Stick: Field-Centric Drive Forward/Backward & Strafe Left/Right</li>
 *   <li>Right Stick: Turn/Yaw with deadzone filtering</li>
 *   <li>Left Bumper: Toggle Slow Drive Mode (50% speed)</li>
 *   <li>Back Button: Reset Gyro Heading (Re-calibrate Field-Centric forward)</li>
 *   <li>D-pad: Fine micro-stepping adjustments</li>
 * </ul>
 * Operator Controls (Gamepad 2):
 * <ul>
 *   <li>A Button: Toggle Launcher Flywheel ON / OFF</li>
 *   <li>B Button: Toggle Launcher Feeder Continuous ON / OFF</li>
 *   <li>X Button: Toggle Intake and Sweepers ON / OFF</li>
 *   <li>Right Bumper: Spin Launcher Feeder a Quarter Turn Forward (~90°)</li>
 *   <li>Left Bumper: Spin Launcher Feeder a Quarter Turn Backwards (~90°)</li>
 *   <li>D-Pad Up / Down: Adjust Launcher Target Velocity (+/- 50 Ticks/Sec)</li>
 * </ul>
 */
@TeleOp(name = "Telop Field Centric", group = "Iterative Opmode")
public class TelopFieldCentric extends OpMode {

    // Drive Speed Modifiers
    private static final double NORMAL_DRIVE_SPEED = 0.8;
    private static final double SLOW_DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.6;
    private static final double STICK_CURVE_EXPONENT = 2.0; // Quadratic response curve for precision

    // State Variables
    private boolean slowMode = false;
    private boolean launcherActive = false;

    // Robot Hardware Reference & Gamepad Controllers
    private HardwareAndFunctions robot;
    private GamepadController driver;
    private GamepadController operator;
    private Launcher launcher;

    @Override
    public void init() {
        // Instantiate robot helper passing this OpMode
        robot = new HardwareAndFunctions(this);

        // Wrap FTC gamepads into GamepadController instances
        driver = new GamepadController(gamepad1);
        operator = new GamepadController(gamepad2);

        // Initialize Robot Hardware (which initializes launcherSubsystem and intakeSubsystem)
        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();
        robot.initialize(false);

        // Convenience reference to launcher subsystem
        launcher = robot.launcherSubsystem;

        telemetry.addData("Status", "Ready! Touch Play to start driving.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Optional loop while waiting for start after init
    }

    @Override
    public void start() {
        // Called once when START is pressed
    }

    @Override
    public void loop() {
        // --- IMPORTANT: Update Gamepad Controllers at the start of each loop iteration ---
        driver.update();
        operator.update();

        // Update sensor readings and subsystem states
        robot.readSensors();
        launcher.update();

        // ==========================================
        // --- DRIVER CONTROLS (GAMEPAD 1) ---
        // ==========================================

        // 1. Toggle Slow Mode using Left Bumper (Single Press / Rising Edge)
        if (driver.leftBumperOnce()) {
            slowMode = !slowMode;
        }

        // Reset Field-Centric Heading using Back Button
        if (driver.wasJustPressed(GamepadController.Button.BACK)) {
            robot.resetHeading();
        }

        double speedMultiplier = slowMode ? SLOW_DRIVE_SPEED : NORMAL_DRIVE_SPEED;

        // 2. Read Joystick Inputs with Exponential Response Curve & Deadzone
        double driveInput  = -driver.getLeftStickYCurve(STICK_CURVE_EXPONENT) * speedMultiplier;
        double strafeInput = -driver.getLeftStickXCurve(STICK_CURVE_EXPONENT) * speedMultiplier;
        double yaw    = -driver.getRightStickXCurve(STICK_CURVE_EXPONENT) * TURN_SPEED;

        // --- Field-Centric Transformation ---
        double botHeading = Math.toRadians(robot.heading);

        double rotX = strafeInput * Math.cos(-botHeading) - driveInput * Math.sin(-botHeading);
        double rotY = strafeInput * Math.sin(-botHeading) + driveInput * Math.cos(-botHeading);

        // 3. Precise Micro-Stepping using D-Pad (overrides joystick input when pressed)
        if (driver.isDown(GamepadController.Button.DPAD_UP)) {
            rotY = SLOW_DRIVE_SPEED / 2.0;
            rotX = 0;
        } else if (driver.isDown(GamepadController.Button.DPAD_DOWN)) {
            rotY = -SLOW_DRIVE_SPEED / 2.0;
            rotX = 0;
        } else if (driver.isDown(GamepadController.Button.DPAD_LEFT)) {
            rotX = SLOW_DRIVE_SPEED / 2.0;
            rotY = 0;
        } else if (driver.isDown(GamepadController.Button.DPAD_RIGHT)) {
            rotX = -SLOW_DRIVE_SPEED / 2.0;
            rotY = 0;
        }

        // Send movement command to hardware (rotY = drive, rotX = strafe, yaw = turn)
        robot.moveRobot(rotY, rotX, yaw);


        // ==========================================
        // --- OPERATOR CONTROLS (GAMEPAD 2) ---
        // ==========================================

        // 1. Toggle Launcher Flywheel ON / OFF with 'A' Button single press
        if (operator.aOnce()) {
            launcherActive = !launcherActive;
            if (launcherActive) {
                launcher.setLauncherVelocity(Launcher.LAUNCHER_TARGET_VELOCITY);
            } else {
                launcher.stopLauncher();
            }
        }

        // 2. Toggle Launcher Feeder Continuous ON / OFF with 'B' Button single press
        if (operator.bOnce()) {
            launcher.toggleFeeder();
        }

        // 3. Toggle Intake and Sweepers ON / OFF with 'X' Button single press
        if (operator.xOnce()) {
            robot.toggleIntake();
        }

        // 4. Spin launcher feeder a quarter turn forward with Right Bumper single press
        if (operator.rightBumperOnce()) {
            robot.feedQuarterTurn();
        }

        // 5. Spin launcher feeder a quarter turn backwards with Left Bumper single press
        if (operator.leftBumperOnce()) {
            robot.feedQuarterTurnBackwards();
        }

        // 5. Adjust Launcher Target Velocity with D-Pad Up / Down (by +/- 50 ticks/sec)
        if (operator.dpadUpOnce()) {
            launcher.launcherTargetVelocity += 50;
            if (launcherActive) {
                launcher.setLauncherVelocity(launcher.launcherTargetVelocity);
            }
        } else if (operator.dpadDownOnce()) {
            launcher.launcherTargetVelocity = Math.max(0, launcher.launcherTargetVelocity - 50);
            if (launcherActive) {
                launcher.setLauncherVelocity(launcher.launcherTargetVelocity);
            }
        }


        // ==========================================
        // --- TELEMETRY / DIAGNOSTICS ---
        // ==========================================

        telemetry.addData("--- Odometry Position ---", "");
        telemetry.addData("Odo X, Y (in)", "X: %.2f | Y: %.2f", robot.driveDistance, robot.strafeDistance);
        telemetry.addData("Heading", "%.1f deg (Field-Centric)", robot.heading);

        telemetry.addData("--- Driver (Gamepad 1) ---", "");
        telemetry.addData("Drive Mode", "FIELD-CENTRIC (Press BACK to reset heading)");
        telemetry.addData("Slow Mode", slowMode ? "ENABLED (50%)" : "DISABLED (100%)");
        telemetry.addData("Drive Powers", "Fwd: %.2f | Str: %.2f | Yaw: %.2f", rotY, rotX, yaw);

        telemetry.addData("--- Operator (Gamepad 2) ---", "");
        telemetry.addData("Launcher Status", launcherActive ? "RUNNING" : "STOPPED");
        telemetry.addData("Feeder Status", launcher.isFeederActive() ? "RUNNING" : "STOPPED");
        telemetry.addData("Intake Status", robot.intakeSubsystem.isIntakeActive() ? "RUNNING" : "STOPPED");
        telemetry.addData("Target Velocity", "%.0f Ticks/Sec", launcher.launcherTargetVelocity);
        telemetry.addData("Current Velocity", "%.0f Ticks/Sec", launcher.getVelocity());

        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.stopRobot();
            if (launcher != null) {
                launcher.stopLauncher();
            }
        }
    }
}
