/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */


package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class HardwareAndFunctions {
    private final boolean INVERT_DRIVE_ODOMETRY  = false;       //  When driving FORWARD, the odometry value MUST increase.  If it does not, flip the value of this constant.
    private final boolean INVERT_STRAFE_ODOMETRY = false;       //  When strafing to the LEFT, the odometry value MUST increase.  If it does not, flip the value of this constant.
    public final double DRIVE_GAIN          = .06;    // Strength of axial position control
    public final double DRIVE_ACCEL         = 10;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    public final double DRIVE_TOLERANCE     = .5;     // Controller is is "inPosition" if position error is < +/- this amount
    public final double DRIVE_DEADBAND      = 0.4;     // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    public final double DRIVE_MAX_AUTO      = 1;     // "default" Maximum Axial power limit during autonomous

    public final double STRAFE_GAIN		 = 0.06;	// Strength of lateral position control
    public final double STRAFE_ACCEL		= 10;	 // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    public final double STRAFE_TOLERANCE	= .5;	 // Controller is is "inPosition" if position error is < +/- this amount
    public final double STRAFE_DEADBAND	 = 0.2;	 // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    public final double STRAFE_MAX_AUTO	 = .6;	 // "default" Maximum Lateral power limit during autonomous

    private final  double YAW_GAIN            = 0.018;    // Strength of Yaw position control
    private final  double YAW_ACCEL           = 6.0;     // Acceleration limit.  Percent Power change per second.  1.0 = 0-100% power in 1 sec.
    private final  double YAW_TOLERANCE       = .25;     // Controller is is "inPosition" if position error is < +/- this amount
    private final  double YAW_DEADBAND        = 0.25;    // Error less than this causes zero output.  Must be smaller than DRIVE_TOLERANCE
    private final  double YAW_MAX_AUTO        = 1.0;     // "default" Maximum Yaw power limit during autonomous

    // Dedicated Subsystems
    public Launcher launcherSubsystem = new Launcher();
    public Intake intakeSubsystem = new Intake();

    // Public Members
    public double driveDistance     = 0; // scaled axial distance (+ = forward)
    public double strafeDistance    = 0; // scaled lateral distance (+ = left)
    public double heading           = 0; // Latest Robot heading from IMU

    // Establish a proportional controller for each axis to calculate the required power to achieve a setpoint.
    public ProportionalControl driveController     = new ProportionalControl(DRIVE_GAIN, DRIVE_ACCEL, DRIVE_MAX_AUTO, DRIVE_TOLERANCE, DRIVE_DEADBAND, false);
    public ProportionalControl strafeController    = new ProportionalControl(STRAFE_GAIN, STRAFE_ACCEL, STRAFE_MAX_AUTO, STRAFE_TOLERANCE, STRAFE_DEADBAND, false);
    public ProportionalControl yawController       = new ProportionalControl(YAW_GAIN, YAW_ACCEL, YAW_MAX_AUTO, YAW_TOLERANCE,YAW_DEADBAND, true);

    // ---  Private Members

    // Hardware interface Objects
    private DcMotorEx leftFrontDrive;     //  control the left front drive wheel
    private DcMotorEx rightFrontDrive;    //  control the right front drive wheel
    private DcMotorEx leftBackDrive;      //  control the left back drive wheel
    private DcMotorEx rightBackDrive;     //  control the right back drive wheel

    private LinearOpMode myOpMode;
    public IMU imu;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.
    private double rawHeading       = 0; // Unmodified heading (degrees)
    private double headingOffset    = 0; // Used to offset heading

    private double turnRate           = 0; // Latest Robot Turn Rate from IMU
    private boolean showTelemetry     = false;

    boolean telop;
    private AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    public List<AprilTagDetection> detectedTags = new ArrayList<>();
    public AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // Robot Constructor
    public HardwareAndFunctions(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {

        if (myOpMode.getClass().isAnnotationPresent(TeleOp.class)) {
            telop = true;
            } else {
            telop = false;
        }

        // Initialize the dedicated Launcher & Intake Subsystem
        launcherSubsystem.init(myOpMode.hardwareMap);
        intakeSubsystem.init(myOpMode.hardwareMap);

        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.

        // !!!  Set the drive direction to ensure positive power drives each wheel forward.
        leftFrontDrive  = setupDriveMotor("lf", DcMotorEx.Direction.REVERSE);   //Control Hub 3
        rightFrontDrive = setupDriveMotor("rf", DcMotorEx.Direction.FORWARD);  //Control Hub 2
        leftBackDrive  = setupDriveMotor( "lb", DcMotorEx.Direction.REVERSE);    //Control Hub 1
        rightBackDrive = setupDriveMotor( "rb",DcMotorEx.Direction.FORWARD);    //Control Hub 0

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Tell the software how the Control Hub is mounted on the robot to align the IMU XYZ axes correctly
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                             RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        initGoBildaPinpointOdometry();

        // zero out all the odometry readings.
        resetOdometry();

        // reset the yaw each time, this makes sure robot doesn't go off in a wonky direction if you don't power off between runs
        imu.resetYaw();

        // Set the desired telemetry state
        this.showTelemetry = showTelemetry;
    }

    /**
     *   Setup a drive motor with passed parameters.  Ensure encoder is reset.
     * @param deviceName  Text name associated with motor in Robot Configuration
     * @param direction   Desired direction to make the wheel run FORWARD with positive power input
     * @return the DcMotor object
     */
    private DcMotorEx setupDriveMotor(String deviceName, DcMotorEx.Direction direction) {
        DcMotorEx aMotor = myOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    private void initGoBildaPinpointOdometry(){

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = myOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(6.25, -7.75,DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }


    /**
     * Read all input devices to determine the robot's motion
     * always return true so this can be used in "while" loop conditions
     * @return true
     */
    public boolean readSensors() {

        if( odo == null ){
            return true;
        }

		/*
			Request an update from the Pinpoint odometry computer. This checks almost all outputs
			from the device in a single I2C read.
		*/
        odo.update();

        // Get the latest position, which includes the x and y coordinates, plus the
        // heading angle
        Pose2D pos = odo.getPosition();

        driveDistance =  pos.getX(DistanceUnit.INCH) * (INVERT_DRIVE_ODOMETRY ? -1 : 1);   // direction forward/back
        strafeDistance = pos.getY(DistanceUnit.INCH) * (INVERT_STRAFE_ODOMETRY ? -1 : 1);   // direction left/right

        heading = getLiveHeading();
        turnRate = getAngularVelocityOtos();

        if (showTelemetry) {
            myOpMode.telemetry.addData("Position", "X: %5.2f, Y: %5.2f", driveDistance, strafeDistance);
            myOpMode.telemetry.addData("Heading", "Degree: %5.2f, Turn rate: %5.2f", heading, turnRate);
            myOpMode.telemetry.addData("Velocity", launcherSubsystem.getVelocity());
          }
        return true;  // do this so this function can be included in the condition for a while loop to keep values fresh.
    }

    //  ########################  Mid level control functions.  #############################3#

    /**
     * Drive in the axial (forward/reverse) direction, maintain the current heading and don't drift sideways
     * @param distanceInches  Distance to travel.  +ve = forward, -ve = reverse.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void drive(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(distanceInches, power);   // achieve desired drive distance
        strafeController.reset(0);              // Maintain zero strafe drift
        yawController.reset();                          // Maintain last turn heading
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (driveController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Strafe in the lateral (left/right) direction, maintain the current heading and don't drift fwd/bwd
     * @param distanceInches  Distance to travel.  +ve = left, -ve = right.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void strafe(double distanceInches, double power, double holdTime) {
        resetOdometry();

        driveController.reset(0.0);             //  Maintain zero drive drift
        strafeController.reset(distanceInches, power);  // Achieve desired Strafe distance
        yawController.reset();                          // Maintain last turn angle
        holdTimer.reset();

        while (myOpMode.opModeIsActive() && readSensors()){

            // implement desired axis powers
            moveRobot(driveController.getOutput(driveDistance), strafeController.getOutput(strafeDistance), yawController.getOutput(heading));

            // Time to exit?
            if (strafeController.inPosition() && yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }

    /**
     * Rotate to an absolute heading/direction
     * @param headingDeg  Heading to obtain.  +ve = CCW, -ve = CW.
     * @param power Maximum power to apply.  This number should always be positive.
     * @param holdTime Minimum time (sec) required to hold the final position.  0 = no hold.
     */
    public void turnTo(double headingDeg, double power, double holdTime) {

        yawController.reset(headingDeg, power);
        while (myOpMode.opModeIsActive() && readSensors()) {

            // implement desired axis powers
            moveRobot(0, 0, yawController.getOutput(heading));

            // Time to exit?
            if (yawController.inPosition()) {
                if (holdTimer.time() > holdTime) {
                    break;   // Exit loop if we are in position, and have been there long enough.
                }
            } else {
                holdTimer.reset();
            }
            myOpMode.sleep(10);
        }
        stopRobot();
    }


    //  ########################  Low level control functions.  ###############################

    /**
     * Drive the wheel motors to obtain the requested axes motions
     * @param drive     Fwd/Rev axis power
     * @param strafe    Left/Right axis power
     * @param yaw       Yaw axis power
     */
    public void moveRobot(double drive, double strafe, double yaw){

        double lF = drive - strafe - yaw;
        double rF = drive + strafe + yaw;
        double lB = drive + strafe - yaw;
        double rB = drive - strafe + yaw;

        double max = Math.max(Math.abs(lF), Math.abs(rF));
        max = Math.max(max, Math.abs(lB));
        max = Math.max(max, Math.abs(rB));

        //normalize the motor values
        if (max > 1.0)  {
            lF /= max;
            rF /= max;
            lB /= max;
            rB /= max;
        }

        //send power to the motors
        leftFrontDrive.setPower(lF);
        rightFrontDrive.setPower(rF);
        leftBackDrive.setPower(lB);
        rightBackDrive.setPower(rB);

        if (showTelemetry) {
            myOpMode.telemetry.addData("Axes D:S:Y", "%5.2f %5.2f %5.2f", drive, strafe, yaw);
            myOpMode.telemetry.addData("Wheels lf:rf:lb:rb", "%5.2f %5.2f %5.2f %5.2f", lF, rF, lB, rB);
            myOpMode.telemetry.update(); //  Assume this is the last thing done in the loop.
        }
    }

    /**
     * Stop all motors.
     */
    public void stopRobot() {
        moveRobot(0,0,0);
    }

    private double getAngularVelocityOtos(){
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        return angularVelocity.zRotationRate;
    }

    private double getLiveHeading(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        rawHeading  = orientation.getYaw(AngleUnit.DEGREES);
        return rawHeading;
    }


    /**
     * Set odometry counts and distances to zero.
     */
    public void resetOdometry() {
        //  TIM'S CODE
        resetTracking();

        readSensors();

        driveDistance = 0.0;
        driveController.reset(0);

        strafeDistance = 0.0;
        strafeController.reset(0);
    }

    private void resetTracking(){
        Pose2D pose2D = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES,0.0 );
        odo.setPosition( pose2D );
    }

    /**
     * Reset the robot heading to zero degrees, and also lock that heading into heading controller.
     */
    public void resetHeading() {
        readSensors();
        headingOffset = rawHeading;
        yawController.reset(0);
        heading = 0;
    }

    public double getHeading() {return heading;}
    public double getTurnRate() {return turnRate;}

    public void autoCorrectForShooting(AprilTagWebcam aprilTagWebcam, int desiredTagId, double desiredDistance) {

        double DISTANCE_TOLERANCE = 1.0;
        double BEARING_TOLERANCE = .5;
        boolean target_found = false;

        double drive;
        double turn;
        double strafe;

        double SPEED_GAIN = 0.02;
        double STRAFE_GAIN = 0.015;
        double TURN_GAIN = 0.01;

        double MAX_AUTO_SPEED = 0.8;
        double MAX_AUTO_STRAFE = 0.8;
        double MAX_AUTO_TURN = 0.3;

        // Use a timer to prevent getting stuck in an infinite loop.
        ElapsedTime timeoutTimer = new ElapsedTime();
        timeoutTimer.reset();
        final double TIMEOUT_SECONDS = 3.0; // Give up after 3 seconds

        // Start the loop. Exit if the opmode is stopped or if we time out.
        while (myOpMode.opModeIsActive() && !target_found  && timeoutTimer.seconds() < TIMEOUT_SECONDS) {
            // Get the LATEST detection data in every loop iteration.
            aprilTagWebcam.update();
            AprilTagDetection currentDetection = aprilTagWebcam.getTagBySpecificId(desiredTagId);

            // Make sure the tag is still visible.
            if (currentDetection != null) {
                // Now, calculate errors using the NEW, updated data.
                double rangeError = (currentDetection.ftcPose.range - desiredDistance);
                double headingError = currentDetection.ftcPose.bearing;
                double yawError = currentDetection.ftcPose.yaw;

                // Check if we have arrived at the target.
                if ((Math.abs(rangeError) <= DISTANCE_TOLERANCE) && (Math.abs(headingError) <= BEARING_TOLERANCE)) {
                    stopRobot();
                    target_found = true; // Set flag to exit the loop

                } else {
                    // Not there yet, so calculate motor powers.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    moveRobot(drive, strafe, turn); // Apply power
                }
            }
        }
        // Ensure the robot is stopped when the loop finishes for any reason.
        stopRobot();
    }

    /**
     * Force the launcher and feeder to return to the IDLE state.
     */
    public void resetFeederTimer() {
        launcherSubsystem.resetFeederTimer();
    }

    /**
     * Sets the velocity of our launcher wheel to the desired speed for shooting
     * @param velocity 0 to Max speed Constant for the position we shoot from
     */
    public void setLauncherVelocity(double velocity){
        launcherSubsystem.setLauncherVelocity(velocity);
    }

    public void stopLauncher(){
        launcherSubsystem.stopLauncher();
    }

    public void turnOnFeeder() {
        launcherSubsystem.turnOnFeeder();
    }

    public void stopFeeder() {
        launcherSubsystem.stopFeeder();
    }

    public void toggleFeeder() {
        launcherSubsystem.toggleFeeder();
    }

    /**
     * Used to launch and queue up balls
     */
    public void launch() {
        launcherSubsystem.launch();
    }

    public int getLaunchCount() {
        return launcherSubsystem.getLaunchCount();
    }

    public void setLaunchCountZero() {
        launcherSubsystem.setLaunchCountZero();
    }

    // --- Intake Subsystem Methods ---

    public void setIntakePower(double power) {
        intakeSubsystem.setPower(power);
    }

    public void turnOnIntake() {
        intakeSubsystem.turnOn();
    }

    public void stopIntake() {
        intakeSubsystem.stop();
    }

    public void toggleIntake() {
        intakeSubsystem.toggle();
    }
}

//****************************************************************************************************
//****************************************************************************************************

/***
 * This class is used to implement a proportional controller which can calculate the desired output power
 * to get an axis to the desired setpoint value.
 * It also implements an acceleration limit, and a max power output.
 */
class ProportionalControl {
    double  lastOutput;
    double  gain;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    ElapsedTime cycleTime = new ElapsedTime();

    public ProportionalControl(double gain, double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = gain;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        reset(0.0);
    }

    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        double error = setPoint - input;
        double dV = cycleTime.seconds() * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {return setPoint;}

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
    }
}
