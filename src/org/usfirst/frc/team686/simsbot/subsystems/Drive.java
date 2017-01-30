package org.usfirst.frc.team686.simsbot.subsystems;

//import java.util.Set;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team686.lib.sensors.BNO055;
import org.usfirst.frc.team686.lib.util.AdaptivePurePursuitController;
import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Path;
import org.usfirst.frc.team686.lib.util.RigidTransform2d;
import org.usfirst.frc.team686.lib.util.Rotation2d;
import org.usfirst.frc.team686.lib.util.SynchronousPID;
import org.usfirst.frc.team686.lib.util.Util;

import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.DataLogger;
import org.usfirst.frc.team686.simsbot.Kinematics;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.loops.Loop;
import org.usfirst.frc.team686.simsbot.subsystems.Drive.DriveControlState;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem 
{
	protected static final int kVelocityControlSlot = 0;
	protected static final int kBaseLockControlSlot = 1;

	private static Drive instance_ = new Drive();

	public static Drive getInstance() { return instance_; }

	private double mLastHeadingErrorDegrees;

	// The robot drivetrain's various states
	public enum DriveControlState
	{
		OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
	}

	public final CANTalon lMotor_, rMotor_;
	private double lMotorCtrl, rMotorCtrl;

	private boolean isBrakeMode_ = true;

	private DriveControlState driveControlState_ ;
	private VelocityHeadingSetpoint velocityHeadingSetpoint_;
	private AdaptivePurePursuitController pathFollowingController_;
	private SynchronousPID velocityHeadingPid_;

	private static BNO055 imu = BNO055.getInstance(Constants.BNO055_PORT);

	// The main control loop (an implementation of Loop), which cycles
	// through different robot states
	private final Loop mLoop=new Loop()
	{
		@Override public void onStart()
		{
			setOpenLoop(DriveSignal.NEUTRAL);
			pathFollowingController_=null;
			setBrakeMode(false);
			// stopOnNextCount_ = false;
		}

		@Override public void onLoop()
		{
			synchronized(Drive.this)
			{
				/*
				 * if (stopOnNextCount_ && getSeesLineCount() > lastSeesLineCount_) 
				 * {
				 * 		poseWhenStoppedOnLine_ =
				 * 		RobotState.getInstance().getLatestFieldToVehicle().getValue();
				 * 		stopOnNextCount_ = false; 
				 * 		stop(); 
				 * }
				 */
	
				switch (driveControlState_)
				{
					case OPEN_LOOP:
						return;
					case BASE_LOCKED:
						return;
					case VELOCITY_SETPOINT:
						// Talons are updating the control loop state
						return;
					case VELOCITY_HEADING_CONTROL:
						updateVelocityHeadingSetpoint();
						return;
					case PATH_FOLLOWING_CONTROL:
						updatePathFollower();
						if(isFinishedPath())
						{
							stop();
						}
						break;
					default:
						System.out.println("Unexpected drive control state: "+driveControlState_);
						break;
				}
			}
		}
	
		@Override public void onStop()
		{
			setOpenLoop(DriveSignal.NEUTRAL);
		}
	};

	// The constructor instantiates all of the drivetrain components when the
	// robot powers up
	private Drive() 
	{
		lMotor_ = new CANTalon(Constants.kLeftMotorTalonId);
		rMotor_ = new CANTalon(Constants.kRightMotorTalonId);

		// Get status at 100Hz
		lMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

		// Start in open loop mode
		lMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		rMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		setLeftRightPower(0, 0);
		setBrakeMode(false);

		// Set up the encoders
		lMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lMotor_.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);
		lMotor_.setInverted(false);
		lMotor_.reverseSensor(false);
		lMotor_.reverseOutput(false);

		rMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rMotor_.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);
		rMotor_.setInverted(false);
		rMotor_.reverseSensor(true); // inverts feedback in closed loop modes
		rMotor_.reverseOutput(false);

		// Load velocity control gains
		lMotor_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		rMotor_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		// Load base lock control gains
		lMotor_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);
		rMotor_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);

		velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
				Constants.kDriveHeadingVelocityKd);
		velocityHeadingPid_.setOutputRange(-30, 30);

		setOpenLoop(DriveSignal.NEUTRAL);
	}

	public Loop getLoop() {
		return mLoop;
	}

	public void setControlState(DriveControlState newState)
    {
    	if (driveControlState_ != newState)
    	{
            switch (newState)
            {
            	case OPEN_LOOP: 
	                lMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	                rMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	                break;

            	case BASE_LOCKED:
        			lMotor_.setProfile(kBaseLockControlSlot);
        			lMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
        			lMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
        			lMotor_.set(lMotor_.getPosition());

        			rMotor_.setProfile(kBaseLockControlSlot);
        			rMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
        			rMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
        			rMotor_.set(rMotor_.getPosition());

        			setBrakeMode(true);
            		break;
            		
            	case VELOCITY_SETPOINT:
            		configureTalonsForSpeedControl();
            		break;
            		
            	case VELOCITY_HEADING_CONTROL:
        			configureTalonsForSpeedControl();
        			velocityHeadingPid_.reset();
            		break;
            		
            	case PATH_FOLLOWING_CONTROL:
        			configureTalonsForSpeedControl();
        			velocityHeadingPid_.reset();
            		break;
            		
            	default:
            		break;
            }
            // set new state
    		driveControlState_ = newState;
    	}
    }

	private void configureTalonsForSpeedControl() {
		if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL &&
			driveControlState_ != DriveControlState.VELOCITY_SETPOINT &&
			driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL)
		{
			lMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
			lMotor_.setProfile(kVelocityControlSlot);
			lMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);

			rMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
			rMotor_.setProfile(kVelocityControlSlot);
			rMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
		}
	}

	
	public synchronized DriveControlState getControlState() {
		return driveControlState_;
	}

	public void testDriveSpeedControl() 
	{
		double  left_inches_per_second = Constants.kDriveWheelCircumInches;
		double right_inches_per_second = Constants.kDriveWheelCircumInches;
		setVelocitySetpoint(left_inches_per_second, right_inches_per_second);
	}

	protected synchronized void setLeftRightPower(double left, double right) 
	{
		// single location to correct inverted motor controls
		lMotorCtrl = +left; // store for logging
		rMotorCtrl = +right;// store for logging
		lMotor_.set(lMotorCtrl);
		rMotor_.set(rMotorCtrl);
	}

	public synchronized void setOpenLoop(DriveSignal signal) {
		setControlState(DriveControlState.OPEN_LOOP);
		setLeftRightPower(signal.lMotor, signal.rMotor);
	}

	public synchronized void setBaseLockOn() {
		setControlState(DriveControlState.BASE_LOCKED);
	}

	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		setControlState(DriveControlState.VELOCITY_SETPOINT);
		updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
	}

	public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
		setControlState(DriveControlState.VELOCITY_HEADING_CONTROL);
		velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec, headingSetpoint);
		updateVelocityHeadingSetpoint();
	}

	/**
	 * The robot follows a set path, which is defined by Waypoint objects.
	 * 
	 * @param Path
	 *            to follow
	 * @param reversed
	 * @see com.team254.lib.util/Path.java
	 */
	public synchronized void followPath(Path path, boolean reversed) 
	{
		setControlState(DriveControlState.PATH_FOLLOWING_CONTROL);
		pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
				Constants.kPathFollowingMaxAccel, Constants.kLoopDt, path, reversed, 1.0);
		updatePathFollower();
	}

	/**
	 * @return Returns if the robot mode is Path Following Control and the set
	 *         path is complete.
	 */
	public synchronized boolean isFinishedPath() {
		return (driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController_.isDone())
				|| driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL;
	}

	public double getLeftDistanceInches() {
		return rotationsToInches(lMotor_.getPosition());
	}

	public double getRightDistanceInches() {
		return rotationsToInches(rMotor_.getPosition());
	}

	public double getLeftVelocityInchesPerSec() {
		return rpmToInchesPerSecond(lMotor_.getSpeed());
	}

	public double getRightVelocityInchesPerSec() {
		return rpmToInchesPerSecond(rMotor_.getSpeed());
	}

	public synchronized void resetEncoders() {
		lMotor_.setPosition(0);
		rMotor_.setPosition(0);

		lMotor_.setEncPosition(0);
		rMotor_.setEncPosition(0);
	}

	public BNO055 getImu() {
		return imu;
	}

	public double getHeading() {
		// measured angle decreases with clockwise rotation
		// it should increase with clockwise rotation (according to
		// documentation, and standard right hand rule convention
		// negate it here to correct
		return -imu.getHeading();
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

	@Override
	public synchronized void zeroSensors() {
		resetEncoders();
	}

	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
				|| driveControlState_ == DriveControlState.VELOCITY_SETPOINT
				|| driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) {
			setLeftRightPower(inchesPerSecondToRpm(left_inches_per_sec), inchesPerSecondToRpm(right_inches_per_sec));
		} else {
			System.out.println("Hit a bad velocity control state");
			setLeftRightPower(0, 0);
		}
	}

	private void updateVelocityHeadingSetpoint() {
		Rotation2d actualGyroAngle = Rotation2d.fromDegrees(getHeading());

		mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse()).getDegrees();

		double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
		updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
				velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
	}

	private void updatePathFollower() {
		RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
		RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
		Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

		// Scale the command to respect the max velocity limits
		double max_vel = 0.0;
		max_vel = Math.max(max_vel, Math.abs(setpoint.left));
		max_vel = Math.max(max_vel, Math.abs(setpoint.right));
		if (max_vel > Constants.kPathFollowingMaxVel) {
			double scaling = Constants.kPathFollowingMaxVel / max_vel;
			setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
		}
		updateVelocitySetpoint(setpoint.left, setpoint.right);
	}

	private static double rotationsToInches(double rotations) {
		return rotations * Constants.kDriveWheelCircumInches;
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}

	private static double inchesToRotations(double inches) {
		return inches / Constants.kDriveWheelCircumInches;
	}

	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	public void setBrakeMode(boolean on) {
		if (isBrakeMode_ != on) {
			lMotor_.enableBrakeMode(on);
			rMotor_.enableBrakeMode(on);
			isBrakeMode_ = on;
		}
	}

	/**
	 * VelocityHeadingSetpoints are used to calculate the robot's path given the
	 * speed of the robot in each wheel and the polar coordinates. Especially
	 * useful if the robot is negotiating a turn and to forecast the robot's
	 * location.
	 */
	public static class VelocityHeadingSetpoint {
		private final double leftSpeed_;
		private final double rightSpeed_;
		private final Rotation2d headingSetpoint_;

		// Constructor for straight line motion
		public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
			leftSpeed_ = leftSpeed;
			rightSpeed_ = rightSpeed;
			headingSetpoint_ = headingSetpoint;
		}

		public double getLeftSpeed() {
			return leftSpeed_;
		}

		public double getRightSpeed() {
			return rightSpeed_;
		}

		public Rotation2d getHeading() {
			return headingSetpoint_;
		}
	}

	@Override
	public void log() {
		DataLogger dataLogger = DataLogger.getInstance();

		dataLogger.putNumber("lMotorCurrent", lMotor_.getOutputCurrent());
		dataLogger.putNumber("rMotorCurrent", rMotor_.getOutputCurrent());
		dataLogger.putNumber("lMotorCtrl", lMotorCtrl);
		dataLogger.putNumber("rMotorCtrl", rMotorCtrl);
		dataLogger.putNumber("lMotorStatus", lMotor_.get());
		dataLogger.putNumber("rMotorStatus", rMotor_.get());
		dataLogger.putNumber("lVelocity", getLeftVelocityInchesPerSec());
		dataLogger.putNumber("rVelocity", getRightVelocityInchesPerSec());
		dataLogger.putNumber("lDistance", getLeftDistanceInches());
		dataLogger.putNumber("rDistance", getRightDistanceInches());
		dataLogger.putNumber("Left PID Error", lMotor_.getClosedLoopError());
		dataLogger.putNumber("Right PID Error", rMotor_.getClosedLoopError());
		dataLogger.putNumber("Heading", getHeading());
		dataLogger.putNumber("Heading Error", mLastHeadingErrorDegrees);
		dataLogger.putNumber("Heading PID Error", velocityHeadingPid_.getError());
		dataLogger.putNumber("Heading PID Output", velocityHeadingPid_.get());

		try // pathFollowingController doesn't exist until started
		{
			pathFollowingController_.log();
		} catch (NullPointerException e) {
			// skip logging pathFollowingController_ when it doesn't exist
		}
	}

}
