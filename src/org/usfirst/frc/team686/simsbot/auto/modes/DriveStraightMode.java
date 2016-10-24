package org.usfirst.frc.team686.simsbot.auto.modes;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class DriveStraightMode extends AutoModeBase {

    public DriveStraightMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Drive Straight");
    	
        runAction(new DriveStraightAction(24.0, 12.0));       		         
    }
}
