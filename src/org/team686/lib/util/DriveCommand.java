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

	public DriveControlMode mode;
	public double left;
    public double right;
    public boolean brake;

    
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

    public void setMode(DriveControlMode _mode) { mode = _mode; }
    public DriveControlMode getMode() { return mode; }
    
    public void setMotors(double _left, double _right) { left = _left; right = _right; }
    public double getLeftMotor()  { return left; }
    public double getRightMotor() { return right; }

    public void setBrake(boolean _brake) { brake = _brake; }
    public boolean getBrake()  { return brake; }
    
    public static DriveCommand NEUTRAL = new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, false);
    public static DriveCommand BRAKE   = new DriveCommand(DriveControlMode.OPEN_LOOP, 0, 0, true);

    @Override
    public String toString() {
        return "L: " + left + ", R: " + right;
    }
}
