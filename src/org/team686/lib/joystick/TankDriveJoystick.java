package org.team686.lib.joystick;

import org.team686.lib.joystick.JoystickControlsBase;
import org.team686.lib.util.DriveCommand;
import org.team686.simsbot.Constants;

/**
 * For use with Xbox steering wheel
 */
public class TankDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new TankDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    
    public DriveCommand getDriveCommand()
    {
    	double lMotorSpeed = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
        double rMotorSpeed = -mStick.getRawAxis(Constants.kXboxRStickYAxis);
	    
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;        
    }
}
