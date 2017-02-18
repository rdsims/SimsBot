package org.team686.simsbot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.AdaptivePurePursuitController;
import org.team686.lib.util.DriveCommand;
import org.team686.lib.util.DriveCommand.DriveControlMode;
import org.team686.lib.util.DriveStatus;
import org.team686.lib.util.Path;
import org.team686.lib.util.Pose;
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
	private static Drive instance = new Drive();
	public static Drive getInstance() { return instance; }

	// drive commands
	private DriveCommand driveCmd = DriveCommand.NEUTRAL;
	private boolean resetEncoderCmd = false;

	// drive status
	public DriveStatus driveStatus;
	
	// velocity heading
	private VelocityHeadingSetpoint velocityHeadingSetpoint = new VelocityHeadingSetpoint();

	// path following
	private AdaptivePurePursuitController pathFollowingController;

	

	// The constructor instantiates all of the drivetrain components when the
	// robot powers up
	private Drive() 
	{
		driveCmd = DriveCommand.NEUTRAL;		
		driveStatus = DriveStatus.getInstance();
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
        	switch (driveCmd.getDriveControlMode())
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
    				updateVelocityHeading();
    				return;
    				
    			case PATH_FOLLOWING:
    				// Need to adjust left/right motor velocities to follow path
    				updatePathFollower();
    				if(isFinishedPath())
    					stop();
    				break;
    				
    			default:
    				System.out.println("Unexpected drive control state: " + driveCmd.getDriveControlMode());
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
		driveCmd.setDriveMode(DriveControlMode.OPEN_LOOP);
		driveCmd.setMotors(cmd.getLeftMotor(), cmd.getRightMotor());
	}

	public void setBaseLockOn() 
	{
		driveCmd.setDriveMode(DriveControlMode.BASE_LOCKED);
	}

	public void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) 
	{
		driveCmd.setDriveMode(DriveControlMode.VELOCITY_SETPOINT);
		driveCmd.setMotors(left_inches_per_sec, right_inches_per_sec);
	}

	public void setVelocityHeadingSetpoint(double forward_inches_per_sec, double headingSetpointDeg) 
	{
		driveCmd.setDriveMode(DriveControlMode.VELOCITY_HEADING);
		velocityHeadingSetpoint = new VelocityHeadingSetpoint(forward_inches_per_sec, headingSetpointDeg);
		velocityHeadingSetpoint.velocityHeadingPID.reset();
		updateVelocityHeading();
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
		private final double speed;
		private final double headingSetpointDeg;

		private SynchronousPID velocityHeadingPID = new SynchronousPID();

		// Constructor for straight line motion
		public VelocityHeadingSetpoint()
		{
			this(0, 0);
		}

		public VelocityHeadingSetpoint(double _speed, double _headingSetpointDeg) 
		{
			speed = _speed;
			headingSetpointDeg = _headingSetpointDeg;
			
			velocityHeadingPID = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi, Constants.kDriveHeadingVelocityKd);
			velocityHeadingPID.setOutputRange(-30, 30);
			
			velocityHeadingPID.setSetpoint(headingSetpointDeg);
		}

		public double getSpeed()  { return speed; }
		public double getHeadingSetpointDeg() { return headingSetpointDeg; }
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
		driveCmd.setDriveMode(DriveControlMode.PATH_FOLLOWING);
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
		if (driveCmd.getDriveControlMode() == DriveControlMode.PATH_FOLLOWING)
			return pathFollowingController.isDone();
		else
			return true;
	}

	private void updatePathFollower() 
	{
// TODO: update AdaptivePurePursuitController to be like VisionDriveAction
// have it call driveCurve()		
		Pose robot_pose = RobotState.getInstance().getLatestFieldToVehicle();
		Pose.Delta command = pathFollowingController.update(robot_pose, Timer.getFPGATimestamp());
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
        
    	Pose.Delta cmd = new Pose.Delta(_robotSpeed, _robotSpeed*_curvature); 
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
    
	private void updateVelocityHeading() 
	{
		// get change in left/right motor speeds based on error in heading
		double deltaSpeed = velocityHeadingSetpoint.velocityHeadingPID.calculate( driveStatus.getHeadingDeg() );
		
		updateVelocitySetpoint(velocityHeadingSetpoint.getSpeed() + deltaSpeed / 2,
				               velocityHeadingSetpoint.getSpeed() - deltaSpeed / 2);
	}

	public void resetVelocityHeadingPID()
	{
		// called from DriveLoop, synchronous with changing into VelocityHeading mode
		velocityHeadingSetpoint.velocityHeadingPID.reset();
		velocityHeadingSetpoint.velocityHeadingPID.setSetpoint(velocityHeadingSetpoint.getHeadingSetpointDeg());
	}
	
	
	/**************************************************************************
	 * VelocitySetpoint code
	 * Configures Talon SRXs to desired left/right wheel velocities
	 *************************************************************************/
	
	private void updateVelocitySetpoint(double _left_inches_per_sec, double _right_inches_per_sec) 
	{
		driveCmd.setMotors(_left_inches_per_sec, _right_inches_per_sec);
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
	public void stop()
	{ 
		setOpenLoop(DriveCommand.NEUTRAL); 
	}

	@Override
	public void zeroSensors() { setResetEncoderCmd(true); }


	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			try // pathFollowingController doesn't exist until started
			{
				put("Drive/DriveControlModeCmd", driveCmd.getDriveControlMode().toString() );
				put("Drive/TalonControlModeCmd", driveCmd.getTalonControlMode().toString() );
				put("Drive/lMotorCmd", driveCmd.getLeftMotor() );
				put("Drive/rMotorCmd", driveCmd.getRightMotor() );
				put("Drive/BrakeModeCmd", driveCmd.getBrake() );
				put("VelocityHeading/PIDError",  velocityHeadingSetpoint.velocityHeadingPID.getError() );
				put("VelocityHeading/PIDOutput", velocityHeadingSetpoint.velocityHeadingPID.get() );

//				AdaptivePurePursuitController.getLogger().log();
			} catch (NullPointerException e) {
				// skip logging pathFollowingController when it doesn't exist
			}
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
}
