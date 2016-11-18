package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class BangBangControlMode extends AutoModeBase 
{
	private int mDemoMode;
	
    public BangBangControlMode(int demoMode) 
    {
    	mDemoMode = demoMode;
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	
    	if (mDemoMode==0)
    	{
        	System.out.println("Starting Auto Mode: Bang Bang (Distance)");
    		runAction(new DistanceBangBangControlAction());
    	}
    	else 
    	{
        	System.out.println("Starting Auto Mode: Bang Bang (Angle)");
    		runAction(new AngleBangBangControlAction());
    	}
    }
}
