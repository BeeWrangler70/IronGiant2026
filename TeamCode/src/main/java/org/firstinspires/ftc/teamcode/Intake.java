package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem class responsible for managing the intake motor and sweepers.
 */
public class Intake {

    public static final double DEFAULT_INTAKE_POWER = 1.0;
    public static final double STOP_POWER = 0.0;

    public DcMotorEx intakeMotor;
    public CRServo leftSweeper;
    public CRServo rightSweeper;

    private boolean intakeActive = false;
    private double targetPower = DEFAULT_INTAKE_POWER;

    /**
     * Initializes the intake motor and sweeper servos using the hardware map.
     *
     * @param hardwareMap FTC OpMode hardwareMap instance.
     */
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftSweeper = hardwareMap.crservo.get("left_sweeper");
        leftSweeper.setDirection(DcMotorSimple.Direction.REVERSE);

        rightSweeper = hardwareMap.crservo.get("right_sweeper");
        rightSweeper.setDirection(DcMotorSimple.Direction.FORWARD);

        stop();
    }

    /**
     * Sets power to the intake motor and both sweepers.
     *
     * @param power Power level (-1.0 to 1.0).
     */
    public void setPower(double power) {
        this.targetPower = power;
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
        if (leftSweeper != null) {
            leftSweeper.setPower(power);
        }
        if (rightSweeper != null) {
            rightSweeper.setPower(power);
        }
        intakeActive = (power != 0.0);
    }

    /**
     * Turns on the intake and sweepers at default power.
     */
    public void turnOn() {
        setPower(DEFAULT_INTAKE_POWER);
    }

    /**
     * Stops the intake and sweepers.
     */
    public void stop() {
        setPower(STOP_POWER);
        intakeActive = false;
    }

    /**
     * Toggles the intake and sweepers ON and OFF.
     */
    public void toggle() {
        if (intakeActive) {
            stop();
        } else {
            turnOn();
        }
    }

    public boolean isIntakeActive() {
        return intakeActive;
    }

    public double getTargetPower() {
        return targetPower;
    }
}
