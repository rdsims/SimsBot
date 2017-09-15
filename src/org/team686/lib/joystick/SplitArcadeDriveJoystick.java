package org.team686.lib.joystick;

import org.team686.lib.joystick.JoystickControlsBase;
import org.team686.lib.util.DriveHelper;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and turn.
 */
public class SplitArcadeDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new SplitArcadeDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    
    public DriveCommand getDriveCommand()
    {
    	double throttle = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
        double turn     = +mStick.getRawAxis(Constants.kXboxRStickXAxis);
    	
	    boolean squaredInputs = true;	// set to true to increase fine control while permitting full power

	    return DriveHelper.getDriveCommand(throttle, turn, squaredInputs);
    }
}
