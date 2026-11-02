package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Standalone OpMode (Iterative OpMode) for testing the launcher motor with gamepad controls.
 * <p>
 * Controls (Gamepad 1):
 * <ul>
 *   <li>A Button: Start Launcher</li>
 *   <li>B Button: Stop Launcher</li>
 *   <li>D-pad Up: Increase Target Velocity (+50 Ticks/Sec)</li>
 *   <li>D-pad Down: Decrease Target Velocity (-50 Ticks/Sec)</li>
 * </ul>
 */
@TeleOp(name = "Launcher Test OpMode - Dash", group = "Test")
public class LauncherTestOpModewFTCDashboard extends OpMode {

    private static final double VELOCITY_STEP = 50.0;
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 3000.0;

    private double targetVelocity = 1250.0; // Starts at 1250
    private DcMotorEx launcher;
    private boolean launcherRunning = false;
    private GamepadController driver;

    @Override
    public void init() {
        driver = new GamepadController(gamepad1);

        telemetry.addData("Status", "Initializing Launcher Test...");
        telemetry.update();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5.0, 0, 0, 12.0));

        telemetry.addData("Status", "Ready! Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driver.update();

        // 1. Start / Stop controls
        if (driver.aOnce()) {
            launcherRunning = true;
        } else if (driver.bOnce()) {
            launcherRunning = false;
        }

        // 2. Adjust target velocity on the fly using D-pad Up and D-pad Down
        if (driver.dpadUpOnce()) {
            targetVelocity = Math.min(targetVelocity + VELOCITY_STEP, MAX_VELOCITY);
        } else if (driver.dpadDownOnce()) {
            targetVelocity = Math.max(targetVelocity - VELOCITY_STEP, MIN_VELOCITY);
        }

        // 3. Apply power/velocity to motor
        if (launcherRunning) {
            launcher.setVelocity(targetVelocity);
        } else {
            launcher.setPower(0.0);
        }

        // 4. Telemetry reporting
        telemetry.addData("Launcher State", launcherRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target Velocity", "%.1f Ticks/Sec", targetVelocity);
        telemetry.addData("Current Velocity", "%.1f Ticks/Sec", launcher.getVelocity());
        telemetry.addData("Motor Power", "%.2f", launcher.getPower());
        telemetry.addData("Controls", "A: Start | B: Stop | D-Pad Up/Down: Adjust Velocity (±50)");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure motor stops on exit
        if (launcher != null) {
            launcher.setPower(0.0);
        }
    }
}
