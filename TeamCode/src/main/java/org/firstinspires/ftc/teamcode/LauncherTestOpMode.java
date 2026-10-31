package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Standalone OpMode for testing the launcher motor with FTC Dashboard support.
 * <p>
 * Features:
 * <ul>
 *   <li>Configurable target velocity starting at 1250 ticks/sec (editable live in FTC Dashboard).</li>
 *   <li>Sends telemetry to both Driver Station and FTC Dashboard.</li>
 *   <li>Gamepad controls to start/stop the launcher.</li>
 *   <li>Annotated with @Disabled so you can easily enable/disable it in the Driver Station.</li>
 *   <li>http://192.168.43.1:8080/dash to access the dashboard</li>
 * </ul>
 */
@Config
@TeleOp(name = "Launcher Test OpMode", group = "Test")
//@Disabled
public class LauncherTestOpMode extends LinearOpMode {

    // FTC Dashboard configurable static variable starting at 1250
    public static double targetVelocity = 1250.0;
    public static double pGain = 5.0;
    public static double iGain = 0.0;
    public static double dGain = 0.0;
    public static double fGain = 12.0;

    private DcMotorEx launcher;
    private boolean launcherRunning = false;

    @Override
    public void runOpMode() {
        // Multi-telemetry for Driver Station + FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        telemetry.addData("Status", "Initializing Launcher Test...");
        telemetry.update();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pGain, iGain, dGain, fGain));

        telemetry.addData("Status", "Ready! Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Gamepad controls: A to start launcher, B to stop launcher
            if (gamepad1.a) {
                launcherRunning = true;
            } else if (gamepad1.b) {
                launcherRunning = false;
            }

            // Update PIDF coefficients if modified in FTC Dashboard
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(pGain, iGain, dGain, fGain));

            if (launcherRunning) {
                launcher.setVelocity(targetVelocity);
            } else {
                launcher.setPower(0.0);
            }

            // Telemetry reporting to Driver Station and FTC Dashboard
            telemetry.addData("Launcher State", launcherRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Target Velocity", "%.1f Ticks/Sec", targetVelocity);
            telemetry.addData("Current Velocity", "%.1f Ticks/Sec", launcher.getVelocity());
            telemetry.addData("Motor Power", "%.2f", launcher.getPower());
            telemetry.addData("PIDF", "P:%.1f I:%.1f D:%.1f F:%.1f", pGain, iGain, dGain, fGain);
            telemetry.addData("Controls", "Gamepad A: Start | Gamepad B: Stop");
            telemetry.update();
        }

        // Ensure motor stops on exit
        if (launcher != null) {
            launcher.setPower(0.0);
        }
    }
}
