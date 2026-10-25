package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Main TeleOp OpMode demonstrating driver control with GamepadController
 * and mechanism control with the new Launcher subsystem.
 * <p>
 * Driver Controls (Gamepad 1):
 * <ul>
 *   <li>Left Stick: Drive Forward/Backward & Strafe Left/Right with exponential curve</li>
 *   <li>Right Stick: Turn/Yaw with deadzone filtering</li>
 *   <li>Left Bumper: Toggle Slow Drive Mode (50% speed)</li>
 *   <li>D-pad: Fine micro-stepping adjustments</li>
 * </ul>
 * Operator Controls (Gamepad 2):
 * <ul>
 *   <li>A Button: Toggle Launcher Flywheel ON / OFF</li>
 *   <li>B Button: Stop Flywheel immediately</li>
 *   <li>X / Y Buttons: Set Flywheel target speeds (Close line vs 3-Point line)</li>
 *   <li>Right Bumper (Pressed Once): Trigger single launch sequence</li>
 * </ul>
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

    // Drive Speed Modifiers
    private static final double NORMAL_DRIVE_SPEED = 0.8;
    private static final double SLOW_DRIVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.6;
    private static final double STICK_CURVE_EXPONENT = 2.0; // Quadratic response curve for precision

    // State Variables
    private boolean slowMode = false;
    private boolean launcherActive = false;

    // Robot Hardware Reference
    private final HardwareAndFunctions robot = new HardwareAndFunctions(this);

    @Override
    public void runOpMode() {

        // Wrap FTC gamepads into GamepadController instances
        GamepadController driver = new GamepadController(gamepad1);
        GamepadController operator = new GamepadController(gamepad2);

        // Initialize Robot Hardware (which initializes launcherSubsystem)
        telemetry.addData("Status", "Initializing Hardware...");
        telemetry.update();
        robot.initialize(false);

        // Convenience reference to launcher subsystem
        Launcher launcher = robot.launcherSubsystem;

        telemetry.addData("Status", "Ready! Touch Play to start driving.");
        telemetry.update();

        // Wait for Start button
        waitForStart();

        while (opModeIsActive()) {

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

            double speedMultiplier = slowMode ? SLOW_DRIVE_SPEED : NORMAL_DRIVE_SPEED;

            // 2. Read Joystick Inputs with Exponential Response Curve & Deadzone
            // Note: Negative sign aligns joystick direction (up on stick = positive forward drive)
            double drive  = -driver.getLeftStickYCurve(STICK_CURVE_EXPONENT) * speedMultiplier;
            double strafe = -driver.getLeftStickXCurve(STICK_CURVE_EXPONENT) * speedMultiplier;
            double yaw    = -driver.getRightStickXCurve(STICK_CURVE_EXPONENT) * TURN_SPEED;

            // 3. Precise Micro-Stepping using D-Pad (overrides joystick input when pressed)
            if (driver.isDown(GamepadController.Button.DPAD_UP)) {
                drive = SLOW_DRIVE_SPEED / 2.0;
            } else if (driver.isDown(GamepadController.Button.DPAD_DOWN)) {
                drive = -SLOW_DRIVE_SPEED / 2.0;
            } else if (driver.isDown(GamepadController.Button.DPAD_LEFT)) {
                strafe = SLOW_DRIVE_SPEED / 2.0;
            } else if (driver.isDown(GamepadController.Button.DPAD_RIGHT)) {
                strafe = -SLOW_DRIVE_SPEED / 2.0;
            }

            // Send movement command to hardware
            robot.moveRobot(drive, strafe, yaw);


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

            // 3. Trigger Single Launch sequence on Right Bumper Single Press
            if (operator.rightBumperOnce()) {
//                launcher.launcherFeeder.setPower(1);
                launcher.leftSweeper.setPower(1);
                launcher.rightSweeper.setPower(1);
            }


            // ==========================================
            // --- TELEMETRY / DIAGNOSTICS ---
            // ==========================================

            telemetry.addData("--- Odometry Position ---", "");
            telemetry.addData("Odo X, Y (in)", "X: %.2f | Y: %.2f", robot.driveDistance, robot.strafeDistance);
            telemetry.addData("Heading", "%.1f deg", robot.heading);

            telemetry.addData("--- Driver (Gamepad 1) ---", "");
            telemetry.addData("Slow Mode", slowMode ? "ENABLED (50%)" : "DISABLED (100%)");
            telemetry.addData("Drive Powers", "Fwd: %.2f | Str: %.2f | Yaw: %.2f", drive, strafe, yaw);

            telemetry.addData("--- Operator (Gamepad 2) ---", "");
            telemetry.addData("Launcher Status", launcherActive ? "RUNNING" : "STOPPED");
            telemetry.addData("Target Velocity", "%.0f Ticks/Sec", launcher.launcherTargetVelocity);
            telemetry.addData("Current Velocity", "%.0f Ticks/Sec", launcher.getVelocity());

            telemetry.update();
        }
    }
}
