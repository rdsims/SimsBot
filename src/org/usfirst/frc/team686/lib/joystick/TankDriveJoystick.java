package org.usfirst.frc.team686.lib.joystick;

import org.usfirst.frc.team686.lib.joystick.JoystickControlsBase;
import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Util;
import org.usfirst.frc.team686.simsbot.Constants;

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

    
    public DriveSignal getDriveSignal()
    {
    	double lMotorSpeed = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
        double rMotorSpeed = -mStick.getRawAxis(Constants.kXboxRStickYAxis);
	    
	    DriveSignal signal = new DriveSignal(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;        
    }
}
