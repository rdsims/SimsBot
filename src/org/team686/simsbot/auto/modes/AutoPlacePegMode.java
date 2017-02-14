package org.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import org.team686.lib.util.Path;
import org.team686.lib.util.Translation2d;
import org.team686.lib.util.Path.Waypoint;
import org.team686.simsbot.Constants;
import org.team686.simsbot.DataLogger;
import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.AutoModeEndedException;
import org.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class AutoPlacePegMode extends AutoModeBase 
{
	List<Waypoint> autoPath1, autoPath2, autoPath3;
	int autoLane;
	
    public AutoPlacePegMode(int lane, boolean isShooting) 
    {
    	Translation2d startPos, prePegPos, pegPos, postPegPos, preShootPos, shootPos, finishPos;
    	
    	autoLane = lane;
    	
		double yScale;
		if (lane == 1)
			yScale = -1.0;
		else if (lane == 2)
			yScale =  0.0;
		else
			yScale = +1.0;
		
/*		
		// use negative x coords because original orientation is assumed 0
		
		startPos        = new Translation2d(-308,  64*yScale);
		if (lane==1 || lane==3)
		{
			prePegPos       = new Translation2d(-208,  64*yScale);
			pegPos          = new Translation2d(-188,  52*yScale);
		}
		else
		{
			prePegPos       = new Translation2d(-232,  0);
			pegPos          = new Translation2d(-196,  0);
		}
		postPegPos      = prePegPos;	// in reverse
		// TODO: zero point turn to new orientation
		if (isShooting)
		{
			if (lane==1)
				preShootPos = new Translation2d(-280,  36);		// go around airship
			else
				preShootPos = postPegPos;						// skip this step
			
			shootPos    = new Translation2d(-280,-126);
		}
		else
		{
			preShootPos = prePegPos;							// skip this step
			shootPos    = prePegPos;							// skip this step
		}
		finishPos       = new Translation2d( -86,-126);
*/
		
		
		
    
		// use negative x coords because original orientation is assumed 0
		
		startPos        = new Translation2d(18, 48*yScale);
		prePegPos       = new Translation2d(60, 48*yScale);
		pegPos          = new Translation2d(96,  0*yScale);
		postPegPos      = prePegPos;	// in reverse
		// TODO: zero point turn to new orientation
		if (lane==1)
			preShootPos = new Translation2d(36, -24);		// go around airship
		else
			preShootPos = new Translation2d(36,  0);
			
		shootPos        = new Translation2d( 36, 48);
		finishPos       = new Translation2d(160, -48);
 

		prePegPos =     prePegPos.translateBy(startPos.inverse());
		pegPos =           pegPos.translateBy(startPos.inverse());
		postPegPos =   postPegPos.translateBy(startPos.inverse());
		preShootPos = preShootPos.translateBy(startPos.inverse());
		shootPos =       shootPos.translateBy(startPos.inverse());
		finishPos =     finishPos.translateBy(startPos.inverse());
		startPos =       startPos.translateBy(startPos.inverse());
		
		
		
    
		double vel = 36;
		
		autoPath1 = new ArrayList<>();
		autoPath1.add(new Waypoint(startPos,   vel));
		autoPath1.add(new Waypoint(prePegPos,  vel));
		autoPath1.add(new Waypoint(pegPos,     vel));
		
		autoPath2 = new ArrayList<>();
		autoPath2.add(new Waypoint(pegPos,     vel));
		autoPath2.add(new Waypoint(prePegPos,  vel));

		autoPath3 = new ArrayList<>();
		autoPath3.add(new Waypoint(prePegPos,  vel));
		if (isShooting)
		{
			autoPath3.add(new Waypoint(preShootPos,vel));
        	autoPath3.add(new Waypoint(shootPos,   vel));
		}
        autoPath3.add(new Waypoint(finishPos,  vel));
    
    }

    public Translation2d getInitialPosition()
    {
    	Waypoint wp = autoPath1.get(0);
    	return wp.position;
    }
    
    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Peg Mode, lane " + autoLane);

    	// drive towards target
//        runAction(new FollowPathAction(new Path(autoPath1), false));   
        // use vision to finish path to target
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
//        runAction(new VisionTurnAction(30));
        // back away from target (note: 2nd parameter is true for reverse)
//        runAction(new FollowPathAction(new Path(autoPath2), true));   
        // take path away from airship
//        runAction(new FollowPathAction(new Path(autoPath3), false));   
    }
    
}
