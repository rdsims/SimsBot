package org.team686.simsbot.subsystems;

//import java.util.Set;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.AdaptivePurePursuitController;
import org.team686.lib.util.DriveSignal;
import org.team686.lib.util.Path;
import org.team686.lib.util.RigidTransform2d;
import org.team686.lib.util.Rotation2d;
import org.team686.lib.util.SynchronousPID;

import org.team686.simsbot.Constants;
import org.team686.simsbot.DataLogger;
import org.team686.simsbot.Kinematics;
import org.team686.simsbot.RobotState;
import org.team686.simsbot.loops.Loop;

// TODO: update to use DriveSignal consistently

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

	double lDistanceInches, rDistanceInches;
	double lSpeedInchesPerSec, rSpeedInchesPerSec;
	double headingRad;

	// motor status
	double lMotorCurrent, rMotorCurrent;
	double lMotorCtrl, rMotorCtrl;
	double lMotorPIDError, rMotorPIDError;
	
	private double mLastHeadingErrorDegrees;

	// The robot drivetrain's various states
	public enum DriveControlState
	{
		OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
	}

	private double lMotorCmd, rMotorCmd;
	private boolean brakeEnableCmd;
	private boolean resetEncoderCmd;

	private DriveControlState driveControlState;
	private VelocityHeadingSetpoint velocityHeadingSetpoint;
	private AdaptivePurePursuitController pathFollowingController;
	private SynchronousPID velocityHeadingPID;



	// The constructor instantiates all of the drivetrain components when the
	// robot powers up
	private Drive() 
	{
		velocityHeadingPID = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi, Constants.kDriveHeadingVelocityKd);
		velocityHeadingPID.setOutputRange(-30, 30);
	}

	
	
	/*
	 * Loop to tend to velocity control loops, where Talon SRXs are monitoring the wheel velocities
	 */
    private final Loop velocityPIDLoop = new Loop() 
    {
        @Override
        public void onStart()
        {
            setOpenLoop(DriveSignal.NEUTRAL);
            pathFollowingController = null;
        }

        @Override
        public void onLoop() 
        {
        	switch (driveControlState)
    		{
    			case OPEN_LOOP:
    			case BASE_LOCKED:
    				// states where Talon SRXs are not controlling velocity
    				return;

    			case VELOCITY_SETPOINT:
    				// Nothing to do: Talons SRXs are updating the control loop state
    				return;
    				
    			case VELOCITY_HEADING_CONTROL:
    				// Need to adjust left/right motor velocities to keep desired heading
    				updateVelocityHeadingSetpoint();
    				return;
    				
    			case PATH_FOLLOWING_CONTROL:
    				// Need to adjust left/right motor velocities to follow path
    				updatePathFollower();
    				if(isFinishedPath())
    				{
    					stop();
    				}
    				break;
    				
    			default:
    				System.out.println("Unexpected drive control state: "+driveControlState);
    				break;
    		}
    	}

        @Override
        public void onStop() 
        {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };

    public Loop getVelocityPIDLoop() { return velocityPIDLoop; }
    
    
    /*
     * Main functions to control motors for each DriveControlState
     */
    
	public synchronized void setOpenLoop(DriveSignal signal) 
	{
		setControlState(DriveControlState.OPEN_LOOP);
		setMotorCmd(signal.lMotor,  signal.rMotor);
		setBrakeEnableCmd(signal.brakeMode);
	}

	public synchronized void setBaseLockOn() 
	{
		setControlState(DriveControlState.BASE_LOCKED);
	}

	public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) 
	{
		setControlState(DriveControlState.VELOCITY_SETPOINT);
		setMotorCmd(left_inches_per_sec,  right_inches_per_sec);
	}

	public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) 
	{
		setControlState(DriveControlState.VELOCITY_HEADING_CONTROL);
		velocityHeadingSetpoint = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec, headingSetpoint);
		updateVelocityHeadingSetpoint();
	}
    

	/*
	 * Set/get functions
	 */
    
	public synchronized void setControlState(DriveControlState newState) { driveControlState = newState; }	// will be picked up by DriveLoop on next iteration
	public synchronized DriveControlState getControlState() { return driveControlState; }

	public synchronized void setMotorCmd(double lVal, double rVal)  { lMotorCmd = lVal; rMotorCmd = rVal; }	// will be picked up by DriveLoop on next iteration

	public synchronized double getLeftMotorCmd()  { return lMotorCmd; }
	public synchronized double getRightMotorCmd() { return rMotorCmd; }

	public synchronized void setBrakeEnableCmd(boolean val) { brakeEnableCmd = val; }
	public synchronized boolean getBrakeEnableCmd() { return brakeEnableCmd; }

	public void resetEncoders() { setResetEncoderCmd(true); }
	
    public synchronized void setResetEncoderCmd(boolean flag) { resetEncoderCmd = flag; }		// will be picked up by DriveLoop on next iteration
    public synchronized boolean getResetEncoderCmd() { return resetEncoderCmd; }
	
	public synchronized void setLeftDistanceInches(double val)  { lDistanceInches = val; }
	public synchronized void setRightDistanceInches(double val) { rDistanceInches = val; }

	public synchronized double getLeftDistanceInches()  { return lDistanceInches; }
	public synchronized double getRightDistanceInches() { return rDistanceInches; }

	public synchronized void setLeftSpeedInchesPerSec(double val)  { lSpeedInchesPerSec = val; }
	public synchronized void setRightSpeedInchesPerSec(double val) { rSpeedInchesPerSec = val; }
	
	public synchronized double getLeftSpeedInchesPerSec()  { return lSpeedInchesPerSec; }
	public synchronized double getRightSpeedInchesPerSec() { return rSpeedInchesPerSec; }

	public synchronized void setHeadingDeg(double val) { setHeadingRad(val*Math.PI/180.0); }
    public synchronized void setHeadingRad(double val) { headingRad = val; }

    public synchronized double getHeadingRad() { return headingRad; };
    public synchronized double getHeadingDeg() { return headingRad*180.0/Math.PI; }
    
	public synchronized void setMotorCurrent(double lVal, double rVal) { lMotorCurrent = lVal; rMotorCurrent = rVal; }
	public synchronized void setMotorCtrl(double lVal, double rVal) { lMotorCtrl = lVal; rMotorCtrl = rVal; }				// current settings, read back from Talon (may be different than commanded values)
	public synchronized void setMotorPIDError(double lVal, double rVal) { lMotorPIDError = lVal; rMotorPIDError = rVal; }
    
	public synchronized double getLeftMotorCurrent()  { return lMotorCurrent; }
	public synchronized double getRightMotorCurrent() { return rMotorCurrent; }

	public synchronized double getLeftMotorCtrl()  { return lMotorCtrl; }
	public synchronized double getRightMotorCtrl() { return rMotorCtrl; }

	public synchronized double getLeftMotorPIDError()  { return lMotorPIDError; }
	public synchronized double getRightMotorPIDError() { return rMotorPIDError; }

	
	
	/**
	 * VelocityHeadingSetpoints are used to calculate the robot's path given the
	 * speed of the robot in each wheel and the polar coordinates. Especially
	 * useful if the robot is negotiating a turn and to forecast the robot's
	 * location.
	 */
	public static class VelocityHeadingSetpoint 
	{
		private final double leftSpeed_;
		private final double rightSpeed_;
		private final Rotation2d headingSetpoint_;

		// Constructor for straight line motion
		public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) 
		{
			leftSpeed_ = leftSpeed;
			rightSpeed_ = rightSpeed;
			headingSetpoint_ = headingSetpoint;
		}

		public double getLeftSpeed() { return leftSpeed_; }
		public double getRightSpeed() {	return rightSpeed_; }
		public Rotation2d getHeading() { return headingSetpoint_; }
	}
	

	
	/**************************************************************************
	 * Path Follower Code
	 * (updates VelocitySetpoints in order to follow a path)
	 *************************************************************************/
	
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
		pathFollowingController = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
				Constants.kPathFollowingMaxAccel, Constants.kLoopDt, path, reversed, 1.0);
		updatePathFollower();
	}

	/**
	 * @return Returns if the robot mode is Path Following Control and the set
	 *         path is complete.
	 */
	public synchronized boolean isFinishedPath() {
		return (driveControlState == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController.isDone())
				|| driveControlState != DriveControlState.PATH_FOLLOWING_CONTROL;
	}

	private void updatePathFollower() 
	{
// TODO: update AdaptivePurePursuitController to be like VisionDriveAction
// have it call driveCurve()		
		RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle();
		RigidTransform2d.Delta command = pathFollowingController.update(robot_pose, Timer.getFPGATimestamp());
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

    
	/**************************************************************************
	 * VelocityHeading code
	 * (updates VelocitySetpoints in order to follow a heading)
	 *************************************************************************/
    
	private void updateVelocityHeadingSetpoint() 
	{
		Rotation2d actualGyroAngle = Rotation2d.fromDegrees(getHeadingDeg());

		mLastHeadingErrorDegrees = velocityHeadingSetpoint.getHeading().rotateBy(actualGyroAngle.inverse()).getDegrees();

		double deltaSpeed = velocityHeadingPID.calculate(mLastHeadingErrorDegrees);
		updateVelocitySetpoint(velocityHeadingSetpoint.getLeftSpeed()  + deltaSpeed / 2,
				               velocityHeadingSetpoint.getRightSpeed() - deltaSpeed / 2);
	}

	public void resetVelocityHeadingPID()
	{
		// called from DriveLoop, synchronous with 
		velocityHeadingPID.reset();
	}
	
	
	/**************************************************************************
	 * VelocitySetpoint code
	 * Configures Talon SRXs to desired left/right wheel velocities
	 *************************************************************************/
	
	private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) 
	{
		if (driveControlState == DriveControlState.VELOCITY_HEADING_CONTROL ||
		    driveControlState == DriveControlState.VELOCITY_SETPOINT ||
		    driveControlState == DriveControlState.PATH_FOLLOWING_CONTROL) 
		{
			setMotorCmd(left_inches_per_sec, right_inches_per_sec);
		} 
		else
		{
			System.out.println("Hit a bad velocity control state");
			stop();
		}
	}


	// test function -- rotates wheels 1 RPM
	public void testDriveSpeedControl() 
	{
		double  left_inches_per_second = Constants.kDriveWheelCircumInches;
		double right_inches_per_second = Constants.kDriveWheelCircumInches;
		setVelocitySetpoint(left_inches_per_second, right_inches_per_second);
	}


	
	
	
	
	/*
	 * Subsystem overrides(non-Javadoc)
	 * @see org.team686.simsbot.subsystems.Subsystem#stop()
	 */
	
	@Override
	public synchronized void stop() { setOpenLoop(DriveSignal.BRAKE); }

	@Override
	public synchronized void zeroSensors() { setResetEncoderCmd(true); }

	@Override
	public void log() {
		DataLogger dataLogger = DataLogger.getInstance();

		dataLogger.putNumber("lMotorCurrent", getLeftMotorCurrent() );
		dataLogger.putNumber("rMotorCurrent", getRightMotorCurrent() );
		dataLogger.putNumber("lMotorCmd", getLeftMotorCmd() );
		dataLogger.putNumber("rMotorCmd", getRightMotorCmd() );
		dataLogger.putNumber("lMotorStatus", getLeftMotorCtrl() );
		dataLogger.putNumber("rMotorStatus", getRightMotorCtrl() );
		dataLogger.putNumber("lVelocity", getLeftSpeedInchesPerSec() );
		dataLogger.putNumber("rVelocity", getRightSpeedInchesPerSec() );
		dataLogger.putNumber("lDistance", getLeftDistanceInches() );
		dataLogger.putNumber("rDistance", getRightDistanceInches() );
		dataLogger.putNumber("Left PID Error", getLeftMotorPIDError() );
		dataLogger.putNumber("Right PID Error", getRightMotorPIDError() );
		dataLogger.putNumber("Heading", getHeadingDeg() );
		dataLogger.putNumber("Heading Error", mLastHeadingErrorDegrees );
		dataLogger.putNumber("Heading PID Error", velocityHeadingPID.getError() );
		dataLogger.putNumber("Heading PID Output", velocityHeadingPID.get() );

		try // pathFollowingController doesn't exist until started
		{
			AdaptivePurePursuitController.log();
		} catch (NullPointerException e) {
			// skip logging pathFollowingController when it doesn't exist
		}
	}
	
}
