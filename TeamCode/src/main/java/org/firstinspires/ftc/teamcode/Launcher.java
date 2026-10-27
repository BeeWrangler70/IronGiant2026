package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Subsystem class responsible for managing the launcher flywheel motor,
 * launcher feeder servo, and ball launching state machine.
 */
public class Launcher {

    // Timing and Speed Constants
    public static double FEED_TIME_SECONDS = 0.5;     // Time (in seconds) to rotate feeder servo ~1/2 turn.
    public static double LAUNCH_DELAY_SECONDS = 0.5;  // Delay before feeding ball to ensure flywheel is stabilized at target speed.
    public static final double STOP_SPEED = 0.0;
    public static final double FEEDER_POWER = 1.0;

    // Target Velocities in Encoder Ticks per Second
    public static double LAUNCHER_TARGET_VELOCITY = 1250;             // Close Shot (~1280 RPM)
    public double launcherTargetVelocity = LAUNCHER_TARGET_VELOCITY;

    // Hardware Interface Objects
    public DcMotorEx launcherMotor;
    public CRServo launcherFeeder;

    private boolean feederActive = false;

    // State Machine
    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    public LaunchState launchState = LaunchState.IDLE;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime launchTimer = new ElapsedTime();

    private int launchCount = 0;
    private boolean firstLaunch = true;

    /**
     * Initializes the launcher flywheel motor and launcher feeder servo using the hardware map.
     *
     * @param hardwareMap FTC OpMode hardwareMap instance.
     */
    public void init(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherMotor.setDirection(DcMotorEx.Direction.FORWARD);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5.0, 0, 0, 12.0));

        launcherFeeder = hardwareMap.crservo.get("launcher_feeder");
        launcherFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        stopLauncher();
        launchState = LaunchState.IDLE;
    }

    /**
     * Call periodically in TeleOp loop to update motor velocity and manage coasting when speed drops.
     */
    public void update() {
        if (launcherMotor == null) return;

        if (launcherTargetVelocity <= 0) {
            launcherMotor.setPower(STOP_SPEED);
            return;
        }

        double currentVel = getVelocity();

        // If spinning significantly faster than target velocity, cut motor power to 0 so the flywheel rapidly coasts down.
        if (currentVel > launcherTargetVelocity + 80) {
            launcherMotor.setPower(0.0);
        } else {
            launcherMotor.setVelocity(launcherTargetVelocity);
        }
    }

    /**
     * Sets the target velocity for the flywheel motor and updates output.
     */
    public void setLauncherVelocity(double velocity) {
        this.launcherTargetVelocity = velocity;
        update();
    }

    /**
     * Gets current flywheel velocity in encoder ticks/sec.
     */
    public double getVelocity() {
        return launcherMotor != null ? launcherMotor.getVelocity() : 0.0;
    }

    /**
     * Manually sets power to the launcher feeder CRServo.
     * @param power Power level (-1.0 to 1.0, where 0.0 is stopped).
     */
    public void setFeederPower(double power) {
        if (launcherFeeder != null) {
            launcherFeeder.setPower(power);
        }
        feederActive = (power != 0.0);
    }

    /**
     * Turns on the launcher feeder servo at default power.
     */
    public void turnOnFeeder() {
        setFeederPower(FEEDER_POWER);
    }

    /**
     * Stops the launcher feeder servo.
     */
    public void stopFeeder() {
        setFeederPower(STOP_SPEED);
    }

    /**
     * Toggles the launcher feeder servo ON and OFF.
     */
    public void toggleFeeder() {
        if (feederActive) {
            stopFeeder();
        } else {
            turnOnFeeder();
        }
    }

    /**
     * Returns whether the launcher feeder is currently active.
     */
    public boolean isFeederActive() {
        return feederActive;
    }

    /**
     * Stops the flywheel motor and launcher feeder servo.
     */
    public void stopLauncher() {
        this.launcherTargetVelocity = STOP_SPEED;
        if (launcherMotor != null) launcherMotor.setPower(STOP_SPEED);
        stopFeeder();
        launchState = LaunchState.IDLE;
    }

    /**
     * Resets the feeder state back to IDLE.
     */
    public void resetFeederTimer() {
        launchState = LaunchState.IDLE;
    }

    /**
     * State machine to spin up flywheel, advance launcherFeeder 1/2 turn, and count launches.
     */
    public void launch() {
        switch (launchState) {
            case IDLE:
                launchState = LaunchState.SPIN_UP;
                break;

            case SPIN_UP:
                if (launcherMotor != null) {
                    launcherMotor.setVelocity(launcherTargetVelocity);
                }
                if (getVelocity() >= launcherTargetVelocity - 50) {
                    launchTimer.reset();
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                turnOnFeeder();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() >= FEED_TIME_SECONDS) {
                    launchCount++;
                    stopFeeder();
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public int getLaunchCount() {
        return launchCount;
    }

    public void setLaunchCountZero() {
        launchCount = 0;
    }
}
