package org.team686.simsbot.auto.modes;

import org.team686.lib.util.Pose;
import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.AutoModeEndedException;
import org.team686.simsbot.auto.actions.*;



public class PointTurnTestMode extends AutoModeBase {

    public PointTurnTestMode() 
    {
		initialPose = new Pose(0, 0, 0);
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Point Turn Test");
    	
        runAction(new PointTurnAction(90.0));	// turn left       		         
        runAction(new WaitAction(0.5));       
        
        runAction(new PointTurnAction(0.0));    // return to center     
        runAction(new WaitAction(0.5));       
        
        runAction(new PointTurnAction(-90.0));	// turn right  
        runAction(new WaitAction(0.5));       
        
        runAction(new PointTurnAction(0.0));	// return to center       		         
        runAction(new WaitAction(0.5));       		         
    }
}
