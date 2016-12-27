package org.usfirst.frc.team686.lib.joystick;

import org.usfirst.frc.team686.lib.joystick.JoystickControlsBase;
import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Util;

/**
 * Joystick controls that use the trigger buttons for acceleration/deceleration.
 */
public class TriggerDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new TriggerDriveJoystick();

    public JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    
    public DriveSignal getDriveSignal()
    {
	    boolean squaredInputs = false;
	    
    	double throttle = mStick.getRawAxis(3) - mStick.getRawAxis(2);
        double turn     = mStick.getX();
     		    
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
	    
	    
	    DriveSignal signal = new DriveSignal(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;
    }
}
