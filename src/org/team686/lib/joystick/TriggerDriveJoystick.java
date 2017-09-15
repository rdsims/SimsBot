package org.team686.lib.joystick;

import org.team686.lib.joystick.JoystickControlsBase;
import org.team686.lib.util.DriveHelper;
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
    	double throttle = mStick.getRawAxis(3) - mStick.getRawAxis(2);
    	double turn     = +mStick.getRawAxis(Constants.kXboxLStickXAxis);
    	
	    boolean squaredInputs = false;	// set to true to increase fine control while permitting full power

	    return DriveHelper.getDriveCommand(throttle, turn, squaredInputs);
    }
}
