package org.team686.lib.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.  
 * The command is set by Drive.java, and read by DriveLoop.java, which sends it to the drive motors
 */
public class DriveCommand
{    
	// The robot drivetrain's various states
	public enum DriveControlMode
	{
		OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
	}

	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization	
	private DriveControlMode mode;
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
    	setMode(_mode);
    	setMotors(_left, _right);
    	setBrake(_brake);
    }

    public synchronized void setMode(DriveControlMode _mode) { mode = _mode; }
    public synchronized DriveControlMode getMode() { return mode; }
    
    public synchronized void setMotors(double _left, double _right) { left = _left; right = _right; }
    public synchronized double getLeftMotor()  { return left; }
    public synchronized double getRightMotor() { return right; }

    public synchronized void setBrake(boolean _brake) { brake = _brake; }
    public synchronized boolean getBrake()  { return brake; }
    
    public static DriveCommand NEUTRAL = new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, false);
    public static DriveCommand BRAKE   = new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, true);

    @Override
    public synchronized String toString() {
        return "L: " + left + ", R: " + right;
    }
}
