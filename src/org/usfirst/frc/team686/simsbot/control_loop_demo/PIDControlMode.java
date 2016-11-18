package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class PIDControlMode extends AutoModeBase 
{
	private int mDemoMode;
	private double Kp, Kd, Ki;
	
    public PIDControlMode(int demoMode, double _kp, double _kd, double _ki) 
    {
    	mDemoMode = demoMode;
    	Kp = _kp;
    	Kd = _kd;
    	Ki = _ki;
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	
    	if (mDemoMode==0)
    	{
        	System.out.println("Starting Auto Mode: Proportional+Derivative (Distance)");
    		runAction(new DistancePIDControlAction(Kp, Kd, Ki));
    	}
    	else 
    	{
        	System.out.println("Starting Auto Mode: Proportional+Derivative (Angle)");
    		runAction(new AnglePIDControlAction(Kp, Kd, Ki));
    	}
    }
}
