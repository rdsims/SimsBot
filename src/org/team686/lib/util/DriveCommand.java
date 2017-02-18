package org.team686.lib.util;

import com.ctre.CANTalon.TalonControlMode;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.  
 * The command is set by Drive.java, and read by DriveLoop.java, which sends it to the drive motors
 */

/*
 * Drive.java and DriveLoop.java implement the five differential-drive control modes in the DriveControlMode enumeration
 * 
 * OPEN_LOOP:    		Used in Teleop.  Joystick controls are mapped into left/right motor speeds, using
 *               		the getDriveCommand() functions in lib.jostick.  Motor controllers are placed in %Vbus mode.
 *                
 * BASE_LOCKED:  		The robot attempts to maintain it's current position.  
 *               		Motor controllers use a position PID loop, and brake mode is enabled.
 *                                 
 * VELOCITY_SETPOINT:	The robot's left and right motors attempt to maintain the commanded velocity.  This should
 *                      result in travel with a constant curvature.  Motor controllers use velocity PID loops to
 *                      maintain velocity setpoints.                                
 *                                 
 * VELOCITY_HEADING:	Like VELOCITY_SETPOINT, with the addition that gyro feedback is used to adjust the left/right 
 * 						motor velocity setpoints in order to maintain a given heading.  Useful for traveling in a straight line.                                 
 *                      Motor controllers use velocity PID to maintain velocity setpoints, RoboRIO uses heading PID
 *                      to adjust velocity setpoints to maintain heading. 
 *                                
 * PATH_FOLLOWING:		Like VELOCITY_SETPOINT, with the addition that the RoboRIO is constantly adjusting the left/right 
 * 						motor setpoints to follow a pre-determined path.  The adjustments are not made with a PID,
 * 						but with an adaptive pure pursuit algorithm.
 *                                 
 */

public class DriveCommand
{    
	// The robot drivetrain's various states
	public enum DriveControlMode
	{
		OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING, PATH_FOLLOWING
	}

	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization	
	private DriveControlMode driveMode = DriveControlMode.OPEN_LOOP;
	private TalonControlMode talonMode = TalonControlMode.PercentVbus;
	private double left;
	private double right;
	private boolean brake;

    
    public DriveCommand(double _left, double _right)
    {
        this(DriveControlMode.OPEN_LOOP, _left, _right, false);
    }

    public DriveCommand(double _left, double _right, boolean _brake)
    {
        this(DriveControlMode.OPEN_LOOP, _left, _right, _brake);
    }

    public DriveCommand(DriveControlMode _mode, double _left, double _right, boolean _brake) 
    {
    	setDriveMode(_mode);
    	setMotors(_left, _right);
    	setBrake(_brake);
    }

    public synchronized void setDriveMode(DriveControlMode _driveMode) 
    {
    	driveMode = _driveMode;
    	switch (driveMode)
    	{
    	case OPEN_LOOP:
    		talonMode = TalonControlMode.PercentVbus;
    		break;
    		
    	case BASE_LOCKED:
    		talonMode = TalonControlMode.Position;
    		
    	case VELOCITY_SETPOINT:
    	case PATH_FOLLOWING:
    		talonMode = TalonControlMode.Speed;
    		break;
    		
    	case VELOCITY_HEADING:
    		talonMode = TalonControlMode.Speed;
    		break;
    		
    	default:
    		talonMode = TalonControlMode.Disabled;
    		break;
    	}
    }
    public synchronized DriveControlMode getDriveControlMode() { return driveMode; }
    public synchronized TalonControlMode getTalonControlMode() { return talonMode; }
    
    public synchronized void   setMotors(double _left, double _right) { left = _left; right = _right; }
    public synchronized double getLeftMotor()  { return left; }
    public synchronized double getRightMotor() { return right; }

    public synchronized void    setBrake(boolean _brake) { brake = _brake; }
    public synchronized boolean getBrake()  { return brake; }
    
    // special constant commands
    public static DriveCommand NEUTRAL() { return new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, false); }
    public static DriveCommand BRAKE()   { return new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, true); }
    
    
    @Override
    public synchronized String toString() 
    {
    	return String.format("%s, %s, %s, L/R: (%+7.3f, % 7.3f)", driveMode, talonMode, brake, left, right);
    }
}
