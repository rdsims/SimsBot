package org.team686.simsbot.auto.modes;

import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.AutoModeEndedException;
import org.team686.simsbot.auto.actions.*;



/**
 * Just drive in a straight line, using VelocityHeading mode
 */
public class DriveStraightMode extends AutoModeBase {

    public DriveStraightMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Drive Straight");
    	
        runAction(new DriveStraightAction(48.0, 24.0));       		         
    }
}
