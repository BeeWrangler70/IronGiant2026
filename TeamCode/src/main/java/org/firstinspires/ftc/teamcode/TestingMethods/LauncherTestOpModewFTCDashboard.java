package org.firstinspires.ftc.teamcode.TestingMethods;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GamepadController;

/**
 * Standalone OpMode (Iterative OpMode) for testing the launcher motor and feeder
 * with FTC Dashboard and gamepad controls.
 * <p>
 * Controls (Gamepad 1):
 * <ul>
 *   <li>A Button: Start Launcher Flywheel</li>
 *   <li>B Button: Stop Launcher Flywheel</li>
 *   <li>D-pad Up/Down: Adjust Target Velocity (±50 Ticks/Sec)</li>
 *   <li>Right Bumper (RB): Toggle Launcher Feeder ON / OFF</li>
 *   <li>Left Bumper (LB): Spin Launcher Feeder a Quarter Turn (~90°)</li>
 * </ul>
 */
@Config
@TeleOp(name = "Launcher Test OpMode - Dash", group = "Test")
public class LauncherTestOpModewFTCDashboard extends OpMode {

    private static final double VELOCITY_STEP = 50.0;
    private static final double MIN_VELOCITY = 0.0;
    private static final double MAX_VELOCITY = 3000.0;
    private static final double FEEDER_POWER = 1.0;
    public static double FEEDER_QUARTER_TURN_TIME_SECONDS = 0.115;

    // FTC Dashboard configurable static variable starting at 1250
    public static double targetVelocity = 1140;  //(was 1280)
    public static double pGain = 15.0;
    public static double iGain = 0.0;
    public static double dGain = 0.0;
    public static double fGain = 13.6;

    private DcMotorEx launcher;
    private CRServo launcherFeeder;
    private boolean launcherRunning = false;
    private boolean feederActive = false;
    private boolean quarterTurnActive = false;
    private final ElapsedTime quarterTurnTimer = new ElapsedTime();

    private GamepadController driver;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        driver = new GamepadController(gamepad1);
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initializing Launcher Test (Dashboard)...");
        telemetry.update();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotor.Direction.FORWARD );
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pGain, iGain, dGain, fGain));

        launcherFeeder = hardwareMap.crservo.get("launcher_feeder");
        launcherFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Ready! Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        driver.update();

        // 1. Start / Stop launcher flywheel controls
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

        // 4. Feeder Quarter-Turn Timer Management
        if (quarterTurnActive) {
            if (quarterTurnTimer.seconds() >= FEEDER_QUARTER_TURN_TIME_SECONDS) {
                if (launcherFeeder != null) launcherFeeder.setPower(0.0);
                quarterTurnActive = false;
            }
        }

        // 5. Right Bumper: Toggle Feeder ON / OFF
        if (driver.rightBumperOnce()) {
            quarterTurnActive = false;
            feederActive = !feederActive;
            if (launcherFeeder != null) {
                launcherFeeder.setPower(feederActive ? FEEDER_POWER : 0.0);
            }
        }

        // 6. Left Bumper: Spin Feeder Quarter Turn (~90°)
        if (driver.leftBumperOnce()) {
            if (!quarterTurnActive) {
                feederActive = false;
                quarterTurnActive = true;
                quarterTurnTimer.reset();
                if (launcherFeeder != null) {
                    launcherFeeder.setPower(FEEDER_POWER);
                }
            }
        }

        // 7. Driver Station Telemetry reporting
        telemetry.addData("Launcher State", launcherRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Feeder State", (feederActive || quarterTurnActive) ? "RUNNING" : "STOPPED");
        telemetry.addData("Target Velocity", "%.1f Ticks/Sec", targetVelocity);
        telemetry.addData("Current Velocity", "%.1f Ticks/Sec", launcher.getVelocity());
        telemetry.addData("Motor Power", "%.2f", launcher.getPower());
        telemetry.addData("PIDF", "P:%.1f I:%.1f D:%.1f F:%.1f", pGain, iGain, dGain, fGain);
        telemetry.addData("Controls", "A: Start | B: Stop | RB: Toggle Feeder | LB: 1/4 Turn");
        telemetry.update();

        // 8. FTC Dashboard Telemetry Packet (Dashboard panels & graphs)
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Launcher State", launcherRunning ? "RUNNING" : "STOPPED");
        packet.put("Feeder State", (feederActive || quarterTurnActive) ? "RUNNING" : "STOPPED");
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
        // Ensure motor and feeder stop on exit
        if (launcher != null) {
            launcher.setPower(0.0);
        }
        if (launcherFeeder != null) {
            launcherFeeder.setPower(0.0);
        }
    }
}
