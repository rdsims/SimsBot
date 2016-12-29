package org.usfirst.frc.team686.lib.joystick;

import org.usfirst.frc.team686.lib.joystick.JoystickControlsBase;
import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Util;

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
    	double lMotorSpeed = -mStick.getRawAxis(1);
        double rMotorSpeed = -mStick.getRawAxis(5);
	    
	    DriveSignal signal = new DriveSignal(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;        
    }
}
