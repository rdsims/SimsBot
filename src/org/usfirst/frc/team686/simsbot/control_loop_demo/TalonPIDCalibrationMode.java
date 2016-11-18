package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class TalonPIDCalibrationMode extends AutoModeBase 
{
	private int mDemoMode;
	
    public TalonPIDCalibrationMode() 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Talon PID Calibration");
		runAction(new TalonPIDCalibrationAction());
    }
}
