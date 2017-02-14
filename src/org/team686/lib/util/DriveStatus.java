package org.team686.lib.util;

import org.team686.simsbot.DataLogger;
import org.team686.simsbot.loops.DriveLoop;

/**
 * Drivetrain status structure, filled by DriveLoop.java
 */
public class DriveStatus
{
	private static DriveStatus instance = new DriveStatus();
	public static DriveStatus getInstance() { return instance; }	
	
	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization

	private double lDistanceInches, rDistanceInches;
	private double lSpeedInchesPerSec, rSpeedInchesPerSec;
	private double headingRad;
	
	private double lMotorCurrent, rMotorCurrent;
	private double lMotorStatus, rMotorStatus;
	private double lMotorPIDError, rMotorPIDError;
	
	public DriveStatus() {}
	
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

	public synchronized void setHeadingDeg(double val) { setHeadingRad(val*Math.PI/180.0); }
    public synchronized void setHeadingRad(double val) { headingRad = val; }

    public synchronized double getHeadingRad() { return headingRad; };
    public synchronized double getHeadingDeg() { return headingRad*180.0/Math.PI; }
	
    

    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		putNumber("DriveStatus/lMotorCurrent", lMotorCurrent );
    		putNumber("DriveStatus/rMotorCurrent", rMotorCurrent );
    		putNumber("DriveStatus/lMotorStatus", lMotorStatus );
    		putNumber("DriveStatus/rMotorStatus", rMotorStatus );
    		putNumber("DriveStatus/lVelocity", lSpeedInchesPerSec );	// used by RaspberryPi set LED velocity display
    		putNumber("DriveStatus/rVelocity", rSpeedInchesPerSec );	// used by RaspberryPi set LED velocity display
    		putNumber("DriveStatus/lDistance", lDistanceInches );
    		putNumber("DriveStatus/rDistance", rDistanceInches );
    		putNumber("DriveStatus/lPIDError",  lMotorPIDError );
    		putNumber("DriveStatus/rPIDError", rMotorPIDError );
    		putNumber("DriveStatus/Heading", getHeadingDeg() );
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
	
}
