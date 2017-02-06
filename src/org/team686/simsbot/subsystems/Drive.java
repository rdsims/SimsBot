package org.team686.simsbot.subsystems;

//import java.util.Set;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.sensors.BNO055;
import org.team686.lib.util.AdaptivePurePursuitController;
import org.team686.lib.util.DriveSignal;
import org.team686.lib.util.Path;
import org.team686.lib.util.RigidTransform2d;
import org.team686.lib.util.Rotation2d;
import org.team686.lib.util.SynchronousPID;
import org.team686.lib.util.Util;

import org.team686.simsbot.Constants;
import org.team686.simsbot.DataLogger;
import org.team686.simsbot.Kinematics;
import org.team686.simsbot.RobotState;
import org.team686.simsbot.loops.DriveInterface;
import org.team686.simsbot.loops.Loop;
import org.team686.simsbot.subsystems.Drive.DriveControlState;
import org.team686.simsbot.subsystems.Drive.VelocityHeadingSetpoint;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem 
{
	private static Drive instance_ = new Drive();

	public static Drive getInstance() { return instance_; }

    static DriveInterface driveInterface = DriveInterface.getInstance();
	
	private double mLastHeadingErrorDegrees;

	// The robot drivetrain's various states
	public enum DriveControlState
	{
		OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
	}

	private double lMotorCtrl, rMotorCtrl;

	private DriveControlState driveControlState_ ;
	private VelocityHeadingSetpoint velocityHeadingSetpoint_;
	private AdaptivePurePursuitController pathFollowingController_;
	private SynchronousPID velocityHeadingPid_;

	private static BNO055 imu = BNO055.getInstance(Constants.BNO055_PORT);


	// The constructor instantiates all of the drivetrain components when the
	// robot powers up
	private Drive() 
	{
		velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
				Constants.kDriveHeadingVelocityKd);
		velocityHeadingPid_.setOutputRange(-30, 30);
	}

	
	

	
onLoop()
{
	switch (drive.getControlState())
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

	
	public void setControlState(DriveControlState newState)
    {
		driveControlState_ = newState;
    }

	public synchronized DriveControlState getControlState() 
	{
		return driveControlState_;
	}

	public synchronized double getLeftMotorCtrl() { return lMotorCtrl; }
	public synchronized double getRightMotorCtrl() { return rMotorCtrl; }
	
	public void testDriveSpeedControl() 
	{
		double  left_inches_per_second = Constants.kDriveWheelCircumInches;
		double right_inches_per_second = Constants.kDriveWheelCircumInches;
		setVelocitySetpoint(left_inches_per_second, right_inches_per_second);
	}

	public synchronized void setOpenLoop(DriveSignal signal) 
	{
		driveControlState_ = DriveControlState.OPEN_LOOP;
		lMotorCtrl = signal.lMotor;
		rMotorCtrl = signal.rMotor;
	}

	public synchronized void setBaseLockOn() 
	{
		driveControlState_ = DriveControlState.BASE_LOCKED;
	}

	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) 
	{
		driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
		lMotorCtrl = left_inches_per_sec;
		rMotorCtrl = right_inches_per_sec;
	}

	public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) 
	{
		driveControlState_ = DriveControlState.VELOCITY_HEADING_CONTROL;
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

	public double getLeftDistanceInches()  { return rotationsToInches(driveInterface.getLeftDistance());	}
	public double getRightDistanceInches() { return rotationsToInches(driveInterface.getRightDistance()); 	}

	public double getLeftVelocityInchesPerSec()  { return rpmToInchesPerSecond(driveInterface.getLeftSpeed());  }
	public double getRightVelocityInchesPerSec() { return rpmToInchesPerSecond(driveInterface.getRightSpeed());	}
	
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

	private void updatePathFollower() 
	{
// TODO: update AdaptivePurePursuitController to be like VisionDriveAction
// have it call driveCurve()		
		RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle();
		RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
		Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

		// Scale the command to respect the max velocity limits
		double max_vel = 0.0;
		max_vel = Math.max(max_vel, Math.abs(setpoint.left));
		max_vel = Math.max(max_vel, Math.abs(setpoint.right));
		if (max_vel > Constants.kPathFollowingMaxVel) 
		{
			double scaling = Constants.kPathFollowingMaxVel / max_vel;
			setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
		}
		updateVelocitySetpoint(setpoint.left, setpoint.right);
	}

    public void driveCurve(double robotSpeed, double curvature, double wheelSpeedLimit)
    {
        // robotSpeed: desired forward speed of robot
        // curvature: curvature of circle to follow.  Curvature = 1/radius.  positive-->turn right, negative-->turn left
        // maxWheelSpeed: the desired velocity will be scaled so that neither wheel exceeds this speed
        
        RigidTransform2d.Delta cmd = new RigidTransform2d.Delta(robotSpeed, 0, robotSpeed*curvature); 
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(cmd);

        // Scale the command to respect the max wheel velocity limits
        double maxSpeed = Math.max(Math.abs(setpoint.left), Math.abs(setpoint.right));
        if (maxSpeed > wheelSpeedLimit)
            setpoint.scale(wheelSpeedLimit/maxSpeed);
        setVelocitySetpoint(setpoint.left, setpoint.right);
    }
    
	private void updateVelocityHeadingSetpoint() {
		Rotation2d actualGyroAngle = Rotation2d.fromDegrees(getHeading());

		mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse()).getDegrees();

		double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
		updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
				velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
	}

	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
				|| driveControlState_ == DriveControlState.VELOCITY_SETPOINT
				|| driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) 
		{
			setLeftRightPower(inchesPerSecondToRpm(left_inches_per_sec), inchesPerSecondToRpm(right_inches_per_sec));
		} 
		else
		{
			System.out.println("Hit a bad velocity control state");
			setLeftRightPower(0, 0);
		}
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
			AdaptivePurePursuitController.log();
		} catch (NullPointerException e) {
			// skip logging pathFollowingController_ when it doesn't exist
		}
	}

}
