package org.team686.lib.joystick;

import org.team686.lib.joystick.JoystickControlsBase;
import org.team686.lib.util.Util;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;

/**
 * Joystick controls that use the trigger buttons for acceleration/deceleration.
 */
public class TriggerDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new TriggerDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    
    public DriveCommand getDriveCommand()
    {
	    boolean squaredInputs = false;
	    
    	double throttle = mStick.getRawAxis(Constants.kXboxRTriggerAxis) - mStick.getRawAxis(Constants.kXboxLTriggerAxis);
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
	    
	    
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;
    }
}
