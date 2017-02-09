package org.team686.lib.util;

import org.team686.simsbot.DataLogger;

/**
 * Drivetrain status structure, filled by DriveLoop.java
 */
public class DriveStatus
{
	double lDistanceInches, rDistanceInches;
	double lSpeedInchesPerSec, rSpeedInchesPerSec;
	double headingRad;
	
	double lMotorCurrent, rMotorCurrent;
	double lMotorStatus, rMotorStatus;
	double lMotorPIDError, rMotorPIDError;
	
	private DriveStatus() 
	{
	}
	
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
	
    
	public void log() 
	{
		DataLogger dataLogger = DataLogger.getInstance();

		dataLogger.putNumber("lMotorCurrent", lMotorCurrent );
		dataLogger.putNumber("rMotorCurrent", rMotorCurrent );
		dataLogger.putNumber("lMotorStatus", lMotorStatus );
		dataLogger.putNumber("rMotorStatus", rMotorStatus );
		dataLogger.putNumber("lVelocity", lSpeedInchesPerSec );
		dataLogger.putNumber("rVelocity", rSpeedInchesPerSec );
		dataLogger.putNumber("lDistance", lDistanceInches );
		dataLogger.putNumber("rDistance", rDistanceInches );
		dataLogger.putNumber("Left PID Error",  lMotorPIDError );
		dataLogger.putNumber("Right PID Error", rMotorPIDError );
		dataLogger.putNumber("Heading", getHeadingDeg() );
	}
    
}
