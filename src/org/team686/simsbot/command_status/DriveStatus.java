package org.team686.simsbot.command_status;

import org.team686.lib.util.DataLogger;

import com.ctre.CANTalon.TalonControlMode;

/**
 * Drivetrain status structure, filled by DriveLoop.java
 */
public class DriveStatus
{
	private static DriveStatus instance = new DriveStatus();
	public static DriveStatus getInstance() { return instance; }	
	
	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization

	private TalonControlMode talonControlMode = TalonControlMode.Disabled;
	private boolean brakeMode;
	
	private double lDistanceInches, rDistanceInches;
	private double lSpeedInchesPerSec, rSpeedInchesPerSec;
	private double heading;
	
	private double lMotorCurrent, rMotorCurrent;
	private double lMotorStatus, rMotorStatus;
	private double lMotorPIDError, rMotorPIDError;
	
	public DriveStatus() {}
	
	public synchronized void setTalonControlMode(TalonControlMode val) { talonControlMode = val; }
	public synchronized TalonControlMode getTalonControlMode() { return talonControlMode; }
	
	public synchronized void setBrakeMode(boolean val) { brakeMode = val; }
	public synchronized boolean getBrakeMode() { return brakeMode; }
	
	public synchronized void setLeftDistanceInches(double val)  { lDistanceInches = val; }
	public synchronized void setRightDistanceInches(double val) { rDistanceInches = val; }

	public synchronized double getLeftDistanceInches()  { return lDistanceInches; }
	public synchronized double getRightDistanceInches() { return rDistanceInches; }

	public synchronized void setLeftSpeedInchesPerSec(double val)  { lSpeedInchesPerSec = val; }
	public synchronized void setRightSpeedInchesPerSec(double val) { rSpeedInchesPerSec = val; }
	
	public synchronized double getLeftSpeedInchesPerSec()  { return lSpeedInchesPerSec; }
	public synchronized double getRightSpeedInchesPerSec() { return rSpeedInchesPerSec; }

	public synchronized void setMotorCurrent(double lVal, double rVal) { lMotorCurrent = lVal; rMotorCurrent = rVal; }
	public synchronized void setMotorStatus(double lVal, double rVal) { lMotorStatus = lVal; rMotorStatus = rVal; }				// current settings, read back from Talon (may be different than commanded values)
	public synchronized void setMotorPIDError(double lVal, double rVal) { lMotorPIDError = lVal; rMotorPIDError = rVal; }
    
	public synchronized double getLeftMotorCurrent()  { return lMotorCurrent; }
	public synchronized double getRightMotorCurrent() { return rMotorCurrent; }

	public synchronized double getLeftMotorCtrl()  { return lMotorStatus; }
	public synchronized double getRightMotorCtrl() { return rMotorStatus; }

	public synchronized double getLeftMotorPIDError()  { return lMotorPIDError; }
	public synchronized double getRightMotorPIDError() { return rMotorPIDError; }

	public synchronized void setHeadingDeg(double val) { setHeading(val*Math.PI/180.0); }
    public synchronized void setHeading(double val) { heading = val; }

    public synchronized double getHeading() { return heading; };
    public synchronized double getHeadingDeg() { return heading*180.0/Math.PI; }
	
    

    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
        	synchronized (DriveStatus.this)
        	{
	    		put("DriveStatus/TalonControlMode", talonControlMode.toString() );
	    		put("DriveStatus/brakeMode", brakeMode );
	    		put("DriveStatus/lMotorCurrent", lMotorCurrent );
	    		put("DriveStatus/rMotorCurrent", rMotorCurrent );
	    		put("DriveStatus/lMotorStatus", lMotorStatus );
	    		put("DriveStatus/rMotorStatus", rMotorStatus );
	    		put("DriveStatus/lSpeed", lSpeedInchesPerSec );	// used by RaspberryPi set LED velocity display
	    		put("DriveStatus/rSpeed", rSpeedInchesPerSec );	// used by RaspberryPi set LED velocity display
	    		put("DriveStatus/lDistance", lDistanceInches );
	    		put("DriveStatus/rDistance", rDistanceInches );
	    		put("DriveStatus/lPIDError",  lMotorPIDError );
	    		put("DriveStatus/rPIDError", rMotorPIDError );
	    		put("DriveStatus/Heading", getHeadingDeg() );
        	}
        }
    };
    
    public DataLogger getLogger() { return logger; }


    
	
}
