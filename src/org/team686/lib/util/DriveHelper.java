package org.team686.lib.util;

import org.team686.simsbot.command_status.DriveCommand;

public class DriveHelper
{
	private static DriveHelper instance = new DriveHelper();
	public static DriveHelper getInstance() { return instance; }

	public static DriveCommand getDriveCommand(double throttle, double turn, boolean squaredInputs)
	{
	    double moveValue   = Util.limit(throttle, 1.0);
	    double rotateValue = Util.limit(turn,     1.0);
	    double lMotorSpeed, rMotorSpeed;
	    
	    if (squaredInputs) {
	      // square the inputs (while preserving the sign) to increase fine control
	      // while permitting full power
	      if (moveValue >= 0.0) {
	        moveValue = (moveValue * moveValue);
	      } else {
	        moveValue = -(moveValue * moveValue);
	      }
	      if (rotateValue >= 0.0) {
	        rotateValue = (rotateValue * rotateValue);
	      } else {
	        rotateValue = -(rotateValue * rotateValue);
	      }
	    }
	
	    if (rotateValue > 0.0) {
	      lMotorSpeed = moveValue;
	      rMotorSpeed = moveValue-rotateValue*moveValue*2;
	    } else {
	      lMotorSpeed = moveValue+rotateValue*moveValue*2;
	      rMotorSpeed = moveValue;
	    }
	    
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	    return signal;
	}
}