package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A wrapper and utility class for the FTC Gamepad that provides:
 * <ul>
 *   <li>Single-frame button press detection (rising edge / falling edge / toggle).</li>
 *   <li>Custom joystick deadzone filtering and exponential curve scaling.</li>
 *   <li>Analog trigger threshold detection and state tracking.</li>
 * </ul>
 */
public class GamepadController {

    public enum Button {
        A, B, X, Y,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        START, BACK, GUIDE, OPTIONS, SHARE
    }

    private final Gamepad gamepad;

    // Previous frame state snapshot for edge detection
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevLeftBumper, prevRightBumper;
    private boolean prevLeftStickButton, prevRightStickButton;
    private boolean prevStart, prevBack, prevGuide, prevOptions, prevShare;
    private boolean prevLeftTrigger, prevRightTrigger;

    // Current frame state snapshot
    private boolean currA, currB, currX, currY;
    private boolean currDpadUp, currDpadDown, currDpadLeft, currDpadRight;
    private boolean currLeftBumper, currRightBumper;
    private boolean currLeftStickButton, currRightStickButton;
    private boolean currStart, currBack, currGuide, currOptions, currShare;
    private boolean currLeftTrigger, currRightTrigger;

    // Configuration Settings
    private double deadzone = 0.05;
    private double triggerThreshold = 0.25;

    /**
     * Constructs a GamepadController wrapping the specified FTC Gamepad.
     *
     * @param gamepad The FTC Gamepad instance (e.g., gamepad1 or gamepad2).
     */
    public GamepadController(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Updates the current and previous states of all buttons and triggers.
     * MUST be called once at the start of each TeleOp loop iteration.
     */
    public void update() {
        if (gamepad == null) return;

        // Save previous frame state
        prevA = currA;
        prevB = currB;
        prevX = currX;
        prevY = currY;

        prevDpadUp = currDpadUp;
        prevDpadDown = currDpadDown;
        prevDpadLeft = currDpadLeft;
        prevDpadRight = currDpadRight;

        prevLeftBumper = currLeftBumper;
        prevRightBumper = currRightBumper;

        prevLeftStickButton = currLeftStickButton;
        prevRightStickButton = currRightStickButton;

        prevStart = currStart;
        prevBack = currBack;
        prevGuide = currGuide;
        prevOptions = currOptions;
        prevShare = currShare;

        prevLeftTrigger = currLeftTrigger;
        prevRightTrigger = currRightTrigger;

        // Read current frame state
        currA = gamepad.a;
        currB = gamepad.b;
        currX = gamepad.x;
        currY = gamepad.y;

        currDpadUp = gamepad.dpad_up;
        currDpadDown = gamepad.dpad_down;
        currDpadLeft = gamepad.dpad_left;
        currDpadRight = gamepad.dpad_right;

        currLeftBumper = gamepad.left_bumper;
        currRightBumper = gamepad.right_bumper;

        currLeftStickButton = gamepad.left_stick_button;
        currRightStickButton = gamepad.right_stick_button;

        currStart = gamepad.start;
        currBack = gamepad.back;
        currGuide = gamepad.guide;
        currOptions = gamepad.options;
        currShare = gamepad.share;

        currLeftTrigger = gamepad.left_trigger >= triggerThreshold;
        currRightTrigger = gamepad.right_trigger >= triggerThreshold;
    }

    // --- Configuration Methods ---

    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    public void setTriggerThreshold(double threshold) {
        this.triggerThreshold = threshold;
    }

    // --- Raw Gamepad Access ---

    public Gamepad getGamepad() {
        return gamepad;
    }

    // --- Edge Detection & State Query Methods ---

    /**
     * Returns true only on the single frame when the specified button was first pressed (rising edge).
     */
    public boolean wasJustPressed(Button button) {
        switch (button) {
            case A: return currA && !prevA;
            case B: return currB && !prevB;
            case X: return currX && !prevX;
            case Y: return currY && !prevY;
            case DPAD_UP: return currDpadUp && !prevDpadUp;
            case DPAD_DOWN: return currDpadDown && !prevDpadDown;
            case DPAD_LEFT: return currDpadLeft && !prevDpadLeft;
            case DPAD_RIGHT: return currDpadRight && !prevDpadRight;
            case LEFT_BUMPER: return currLeftBumper && !prevLeftBumper;
            case RIGHT_BUMPER: return currRightBumper && !prevRightBumper;
            case LEFT_STICK_BUTTON: return currLeftStickButton && !prevLeftStickButton;
            case RIGHT_STICK_BUTTON: return currRightStickButton && !prevRightStickButton;
            case START: return currStart && !prevStart;
            case BACK: return currBack && !prevBack;
            case GUIDE: return currGuide && !prevGuide;
            case OPTIONS: return currOptions && !prevOptions;
            case SHARE: return currShare && !prevShare;
            default: return false;
        }
    }

    /**
     * Returns true only on the single frame when the specified button was released (falling edge).
     */
    public boolean wasJustReleased(Button button) {
        switch (button) {
            case A: return !currA && prevA;
            case B: return !currB && prevB;
            case X: return !currX && prevX;
            case Y: return !currY && prevY;
            case DPAD_UP: return !currDpadUp && prevDpadUp;
            case DPAD_DOWN: return !currDpadDown && prevDpadDown;
            case DPAD_LEFT: return !currDpadLeft && prevDpadLeft;
            case DPAD_RIGHT: return !currDpadRight && prevDpadRight;
            case LEFT_BUMPER: return !currLeftBumper && prevLeftBumper;
            case RIGHT_BUMPER: return !currRightBumper && prevRightBumper;
            case LEFT_STICK_BUTTON: return !currLeftStickButton && prevLeftStickButton;
            case RIGHT_STICK_BUTTON: return !currRightStickButton && prevRightStickButton;
            case START: return !currStart && prevStart;
            case BACK: return !currBack && prevBack;
            case GUIDE: return !currGuide && prevGuide;
            case OPTIONS: return !currOptions && prevOptions;
            case SHARE: return !currShare && prevShare;
            default: return false;
        }
    }

    /**
     * Returns true continuously while the specified button is held down.
     */
    public boolean isDown(Button button) {
        switch (button) {
            case A: return currA;
            case B: return currB;
            case X: return currX;
            case Y: return currY;
            case DPAD_UP: return currDpadUp;
            case DPAD_DOWN: return currDpadDown;
            case DPAD_LEFT: return currDpadLeft;
            case DPAD_RIGHT: return currDpadRight;
            case LEFT_BUMPER: return currLeftBumper;
            case RIGHT_BUMPER: return currRightBumper;
            case LEFT_STICK_BUTTON: return currLeftStickButton;
            case RIGHT_STICK_BUTTON: return currRightStickButton;
            case START: return currStart;
            case BACK: return currBack;
            case GUIDE: return currGuide;
            case OPTIONS: return currOptions;
            case SHARE: return currShare;
            default: return false;
        }
    }

    // --- Convenience Single-Press (Rising Edge) Helpers ---

    public boolean aOnce() { return wasJustPressed(Button.A); }
    public boolean bOnce() { return wasJustPressed(Button.B); }
    public boolean xOnce() { return wasJustPressed(Button.X); }
    public boolean yOnce() { return wasJustPressed(Button.Y); }
    public boolean dpadUpOnce() { return wasJustPressed(Button.DPAD_UP); }
    public boolean dpadDownOnce() { return wasJustPressed(Button.DPAD_DOWN); }
    public boolean dpadLeftOnce() { return wasJustPressed(Button.DPAD_LEFT); }
    public boolean dpadRightOnce() { return wasJustPressed(Button.DPAD_RIGHT); }
    public boolean leftBumperOnce() { return wasJustPressed(Button.LEFT_BUMPER); }
    public boolean rightBumperOnce() { return wasJustPressed(Button.RIGHT_BUMPER); }

    // Trigger Edge Detection
    public boolean leftTriggerOnce() { return currLeftTrigger && !prevLeftTrigger; }
    public boolean rightTriggerOnce() { return currRightTrigger && !prevRightTrigger; }

    // --- Joystick Inputs with Deadzone ---

    public double getLeftStickX() {
        return applyDeadzone(gamepad != null ? gamepad.left_stick_x : 0.0);
    }

    public double getLeftStickY() {
        return applyDeadzone(gamepad != null ? gamepad.left_stick_y : 0.0);
    }

    public double getRightStickX() {
        return applyDeadzone(gamepad != null ? gamepad.right_stick_x : 0.0);
    }

    public double getRightStickY() {
        return applyDeadzone(gamepad != null ? gamepad.right_stick_y : 0.0);
    }

    // --- Exponential Curve Inputs for Driver Precision ---

    /**
     * Gets left stick X with an exponential curve applied.
     * @param power Curve exponent (e.g., 2.0 for quadratic, 3.0 for cubic).
     */
    public double getLeftStickXCurve(double power) {
        return applyCurve(getLeftStickX(), power);
    }

    /**
     * Gets left stick Y with an exponential curve applied.
     * @param power Curve exponent (e.g., 2.0 for quadratic, 3.0 for cubic).
     */
    public double getLeftStickYCurve(double power) {
        return applyCurve(getLeftStickY(), power);
    }

    /**
     * Gets right stick X with an exponential curve applied.
     * @param power Curve exponent (e.g., 2.0 for quadratic, 3.0 for cubic).
     */
    public double getRightStickXCurve(double power) {
        return applyCurve(getRightStickX(), power);
    }

    /**
     * Gets right stick Y with an exponential curve applied.
     * @param power Curve exponent (e.g., 2.0 for quadratic, 3.0 for cubic).
     */
    public double getRightStickYCurve(double power) {
        return applyCurve(getRightStickY(), power);
    }

    // --- Analog Trigger Inputs ---

    public double getLeftTrigger() {
        return gamepad != null ? gamepad.left_trigger : 0.0;
    }

    public double getRightTrigger() {
        return gamepad != null ? gamepad.right_trigger : 0.0;
    }

    // --- Internal Math Helpers ---

    private double applyDeadzone(double input) {
        if (Math.abs(input) < deadzone) {
            return 0.0;
        }
        return Math.signum(input) * ((Math.abs(input) - deadzone) / (1.0 - deadzone));
    }

    private double applyCurve(double input, double power) {
        return Math.signum(input) * Math.pow(Math.abs(input), power);
    }
}
