package org.team686.simsbot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.AdaptivePurePursuitController;
import org.team686.lib.util.DriveCommand;
import org.team686.lib.util.DriveCommand.DriveControlMode;
import org.team686.lib.util.DriveStatus;
import org.team686.lib.util.Path;
import org.team686.lib.util.RigidTransform2d;
import org.team686.lib.util.Rotation2d;
import org.team686.lib.util.SynchronousPID;

import org.team686.simsbot.Constants;
import org.team686.simsbot.DataLogger;
import org.team686.simsbot.Kinematics;
import org.team686.simsbot.RobotState;
import org.team686.simsbot.loops.Loop;

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

	// drive commands
	private DriveCommand driveCmd;
	private boolean resetEncoderCmd;

	// drive status
	public DriveStatus driveStatus;
	
	// velocity heading
	private VelocityHeadingSetpoint velocityHeadingSetpoint;
	private SynchronousPID velocityHeadingPID;
	private double mLastHeadingErrorDegrees;

	// path following
	private AdaptivePurePursuitController pathFollowingController;

	

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
            setOpenLoop(DriveCommand.NEUTRAL);
            pathFollowingController = null;
        }

        @Override
        public void onLoop() 
        {
        	switch (driveCmd.getMode())
    		{
    			case OPEN_LOOP:
    			case BASE_LOCKED:
    				// states where Talon SRXs are not controlling velocity
    				return;

    			case VELOCITY_SETPOINT:
    				// Nothing to do: Talons SRXs are updating the control loop state
    				return;
    				
    			case VELOCITY_HEADING:
    				// Need to adjust left/right motor velocities to keep desired heading
    				updateVelocityHeadingSetpoint();
    				return;
    				
    			case PATH_FOLLOWING:
    				// Need to adjust left/right motor velocities to follow path
    				updatePathFollower();
    				if(isFinishedPath())
    				{
    					stop();
    				}
    				break;
    				
    			default:
    				System.out.println("Unexpected drive control state: " + driveCmd.getMode());
    				break;
    		}
    	}

        @Override
        public void onStop() 
        {
            setOpenLoop(DriveCommand.NEUTRAL);
        }
    };

    public Loop getVelocityPIDLoop() { return velocityPIDLoop; }
    
    
    /*
     * Main functions to control motors for each DriveControlState
     */
    
	public void setOpenLoop(DriveCommand cmd) 
	{
		cmd.setMode(DriveControlMode.OPEN_LOOP);
		setCommand(cmd);
	}

	public void setBaseLockOn() 
	{
		driveCmd.setMode(DriveControlMode.BASE_LOCKED);
	}

	public void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) 
	{
		driveCmd.setMode(DriveControlMode.VELOCITY_SETPOINT);
		driveCmd.setMotors(left_inches_per_sec, right_inches_per_sec);
	}

	public void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) 
	{
		driveCmd.setMode(DriveControlMode.VELOCITY_HEADING);
		velocityHeadingSetpoint = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec, headingSetpoint);
		updateVelocityHeadingSetpoint();
	}
    

	/*
	 * Set/get functions
	 */
    
	public void setCommand(DriveCommand cmd) { driveCmd = cmd; }
	public DriveCommand getCommand() { return driveCmd; }

	public void resetEncoders() { setResetEncoderCmd(true); }
	
    public void setResetEncoderCmd(boolean flag) { resetEncoderCmd = flag; }		// will be picked up by DriveLoop on next iteration
    public boolean getResetEncoderCmd() { return resetEncoderCmd; }
	

    
	/**
	 * VelocityHeadingSetpoints are used to calculate the robot's path given the
	 * speed of the robot in each wheel and the polar coordinates. Especially
	 * useful if the robot is negotiating a turn and to forecast the robot's
	 * location.
	 */
	public static class VelocityHeadingSetpoint 
	{
		private final double leftSpeed;
		private final double rightSpeed;
		private final Rotation2d headingSetpoint;

		// Constructor for straight line motion
		public VelocityHeadingSetpoint(double _leftSpeed, double _rightSpeed, Rotation2d _headingSetpoint) 
		{
			leftSpeed = _leftSpeed;
			rightSpeed = _rightSpeed;
			headingSetpoint = _headingSetpoint;
		}

		public double getLeftSpeed() { return leftSpeed; }
		public double getRightSpeed() {	return rightSpeed; }
		public Rotation2d getHeading() { return headingSetpoint; }
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
	 * @param _reversed
	 * @see com.team254.lib.util/Path.java
	 */
	public void followPath(Path _path, boolean	_reversed) 
	{
		driveCmd.setMode(DriveControlMode.PATH_FOLLOWING);
		pathFollowingController = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
				Constants.kPathFollowingMaxAccel, Constants.kLoopDt, _path, _reversed, 1.0);
		updatePathFollower();
	}

	/**
	 * @return Returns if the robot mode is Path Following Control and the set
	 *         path is complete.
	 */
	public boolean isFinishedPath()
	{
		if (driveCmd.getMode() == DriveControlMode.PATH_FOLLOWING)
			return pathFollowingController.isDone();
		else
			return true;
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

    public void driveCurve(double _robotSpeed, double _curvature, double _wheelSpeedLimit)
    {
        // robotSpeed: desired forward speed of robot
        // curvature: curvature of circle to follow.  Curvature = 1/radius.  positive-->turn right, negative-->turn left
        // maxWheelSpeed: the desired velocity will be scaled so that neither wheel exceeds this speed
        
        RigidTransform2d.Delta cmd = new RigidTransform2d.Delta(_robotSpeed, 0, _robotSpeed*_curvature); 
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(cmd);

        // Scale the command to respect the max wheel velocity limits
        double maxSpeed = Math.max(Math.abs(setpoint.left), Math.abs(setpoint.right));
        if (maxSpeed > _wheelSpeedLimit)
            setpoint.scale(_wheelSpeedLimit/maxSpeed);
        setVelocitySetpoint(setpoint.left, setpoint.right);
    }

    
	/**************************************************************************
	 * VelocityHeading code
	 * (updates VelocitySetpoints in order to follow a heading)
	 *************************************************************************/
    
	private void updateVelocityHeadingSetpoint() 
	{
		Rotation2d actualGyroAngle = Rotation2d.fromDegrees(driveStatus.getHeadingDeg());

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
	
	private void updateVelocitySetpoint(double _left_inches_per_sec, double _right_inches_per_sec) 
	{
		DriveControlMode mode = driveCmd.getMode();
		
		if (mode == DriveControlMode.VELOCITY_HEADING ||
			mode == DriveControlMode.VELOCITY_SETPOINT ||
			mode == DriveControlMode.PATH_FOLLOWING) 
		{
			driveCmd.setMotors(_left_inches_per_sec, _right_inches_per_sec);
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
	public void stop() { setOpenLoop(DriveCommand.BRAKE); }

	@Override
	public void zeroSensors() { setResetEncoderCmd(true); }

	@Override
	public void log() {
		DataLogger dataLogger = DataLogger.getInstance();

		dataLogger.putString("driveMode", driveCmd.getMode().toString() );
		dataLogger.putNumber("lMotorCmd", driveCmd.getLeftMotor() );
		dataLogger.putNumber("rMotorCmd", driveCmd.getRightMotor() );
		dataLogger.putBoolean("brakeMode", driveCmd.getBrake() );
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
