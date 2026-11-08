package org.firstinspires.ftc.teamcode.TestingMethods;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.GamepadController;

/**
 * Standalone OpMode (Iterative OpMode) for testing the launcher motor with FTC Dashboard and gamepad controls.
 * <p>
 * Features:
 * <ul>
 *   <li>Configurable target velocity starting at 1250 ticks/sec (editable live in FTC Dashboard via @Config).</li>
 *   <li>Sends telemetry packets to FTC Dashboard using TelemetryPacket alongside Driver Station telemetry.</li>
 *   <li>Gamepad controls to start/stop the launcher and adjust velocity.</li>
 * </ul>
 */
@Config
@TeleOp(name = "Launcher Test OpMode - Dash", group = "Test")
public class LauncherTestOpModewFTCDashboard extends OpMode {

    private static final double VELOCITY_STEP = 50.0;
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 3000.0;

    // FTC Dashboard configurable static variable starting at 1250
    public static double targetVelocity = 1250.0;
    public static double pGain = 5.0;
    public static double iGain = 0.0;
    public static double dGain = 0.0;
    public static double fGain = 12.0;

    private DcMotorEx launcher;
    private boolean launcherRunning = false;
    private GamepadController driver;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        driver = new GamepadController(gamepad1);
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initializing Launcher Test (Dashboard)...");
        telemetry.update();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pGain, iGain, dGain, fGain));

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

        // Update PIDF coefficients if modified in FTC Dashboard
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pGain, iGain, dGain, fGain));

        // 3. Apply power/velocity to motor
        if (launcherRunning) {
            launcher.setVelocity(targetVelocity);
        } else {
            launcher.setPower(0.0);
        }

        // 4. Driver Station Telemetry reporting
        telemetry.addData("Launcher State", launcherRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target Velocity", "%.1f Ticks/Sec", targetVelocity);
        telemetry.addData("Current Velocity", "%.1f Ticks/Sec", launcher.getVelocity());
        telemetry.addData("Motor Power", "%.2f", launcher.getPower());
        telemetry.addData("PIDF", "P:%.1f I:%.1f D:%.1f F:%.1f", pGain, iGain, dGain, fGain);
        telemetry.addData("Controls", "Gamepad A: Start | B: Stop | D-Pad Up/Down: Adjust Velocity (±50)");
        telemetry.update();

        // 5. FTC Dashboard Telemetry Packet (Dashboard panels & graphs)
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher State", launcherRunning ? "RUNNING" : "STOPPED");
        packet.put("Target Velocity", targetVelocity);
        packet.put("Current Velocity", launcher.getVelocity());
        packet.put("Motor Power", launcher.getPower());
        packet.put("P Gain", pGain);
        packet.put("I Gain", iGain);
        packet.put("D Gain", dGain);
        packet.put("F Gain", fGain);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        // Ensure motor stops on exit
        if (launcher != null) {
            launcher.setPower(0.0);
        }
    }
}
