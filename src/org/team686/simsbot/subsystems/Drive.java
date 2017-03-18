package org.team686.simsbot.subsystems;

import org.team686.lib.util.DataLogger;
import org.team686.lib.util.Kinematics.WheelSpeed;
import org.team686.lib.util.PIDController;

import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.command_status.DriveStatus;
import org.team686.simsbot.command_status.DriveCommand.DriveControlMode;
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
	private DriveCommand driveCmd;

	// drive status
	public DriveStatus driveStatus;
	
	// velocity heading
	private VelocityHeadingSetpoint velocityHeadingSetpoint = new VelocityHeadingSetpoint();

	

	// The constructor instantiates all of the drivetrain components when the
	// robot powers up
	private Drive() 
	{
		driveCmd = DriveCommand.NEUTRAL();	
		driveStatus = DriveStatus.getInstance();
	}

	
	
	/*
	 * Loop to tend to velocity control loops, where Talon SRXs are monitoring the wheel velocities
	 */
	// TODO: move into VelocityHeading class
    private final Loop velocityControlLoop = new Loop() 
    {
        @Override
        public void onStart()
        {
            setOpenLoop(DriveCommand.NEUTRAL());
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
    				
    			default:
    				System.out.println("Unexpected drive control state: " + driveCmd.getDriveControlMode());
    				break;
    		}
    	}

        @Override
        public void onStop() 
        {
            setOpenLoop(DriveCommand.NEUTRAL());
        }
    };

    public Loop getVelocityPIDLoop() { return velocityControlLoop; }
    
    
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

	public void setVelocitySetpoint(WheelSpeed _wheelSpeed) 
	{
		driveCmd.setDriveMode(DriveControlMode.VELOCITY_SETPOINT);
		driveCmd.setMotors(_wheelSpeed);
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

		private PIDController velocityHeadingPID = new PIDController();

		// Constructor for straight line motion
		public VelocityHeadingSetpoint()
		{
			this(0, 0);
		}

		public VelocityHeadingSetpoint(double _speed, double _headingSetpointDeg) 
		{
			speed = _speed;
			headingSetpointDeg = _headingSetpointDeg;
			
			velocityHeadingPID = new PIDController(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi, Constants.kDriveHeadingVelocityKd);
			velocityHeadingPID.setOutputRange(-30, 30);
			
			velocityHeadingPID.setSetpoint(headingSetpointDeg);
		}

		public double getSpeed()  { return speed; }
		public double getHeadingSetpointDeg() { return headingSetpointDeg; }
	}
	

	
	/**************************************************************************
	 * VelocityHeading code
	 * (updates VelocitySetpoints in order to follow a heading)
	 *************************************************************************/
    
	private void updateVelocityHeading() 
	{
		// get change in left/right motor speeds based on error in heading
		double diffSpeed = velocityHeadingSetpoint.velocityHeadingPID.calculate( driveStatus.getHeadingDeg() );
		
		updateVelocitySetpoint(velocityHeadingSetpoint.getSpeed() + diffSpeed / 2,
				               velocityHeadingSetpoint.getSpeed() - diffSpeed / 2);
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
		setOpenLoop(DriveCommand.NEUTRAL()); 
	}

	@Override
	public void zeroSensors() { driveCmd.setResetEncoders(); }


	
	
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
