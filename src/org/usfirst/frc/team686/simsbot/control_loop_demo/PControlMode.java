package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class PControlMode extends AutoModeBase 
{
	private int mDemoMode;
	private double Kp, Kd, Ki;
	
    public PControlMode(int demoMode, double kp, double kd, double ki) 
    {
    	mDemoMode = demoMode;
    	Kp = kp;
    	Kd = 0;
    	Ki = 0;
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	
    	if (mDemoMode==0)
    	{
        	System.out.println("Starting Auto Mode: Proportional (Distance)");
    		runAction(new DistancePControlAction(Kp));
    	}
    	else 
    	{
        	System.out.println("Starting Auto Mode: Proportional (Angle)");
    		runAction(new AnglePControlAction(Kp));
    	}
    }
}
