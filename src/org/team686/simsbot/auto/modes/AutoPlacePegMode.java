package org.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import org.mini2Dx.gdx.math.Vector2;
import org.team686.lib.util.Path;
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
    	Vector2 startPos, prePegPos, pegPos, postPegPos, preShootPos, shootPos, finishPos;
    	
    	autoLane = lane;
    	
		float yScale;
		if (lane == 1)
			yScale = -1.0f;
		else if (lane == 2)
			yScale =  0.0f;
		else
			yScale = +1.0f;
		
/*		
		// use negative x coords because original orientation is assumed 0
		
		startPos        = new Vector2(-308,  64*yScale);
		if (lane==1 || lane==3)
		{
			prePegPos       = new Vector2(-208,  64*yScale);
			pegPos          = new Vector2(-188,  52*yScale);
		}
		else
		{
			prePegPos       = new Vector2(-232,  0);
			pegPos          = new Vector2(-196,  0);
		}
		postPegPos      = prePegPos;	// in reverse
		// TODO: zero point turn to new orientation
		if (isShooting)
		{
			if (lane==1)
				preShootPos = new Vector2(-280,  36);		// go around airship
			else
				preShootPos = postPegPos;						// skip this step
			
			shootPos    = new Vector2(-280,-126);
		}
		else
		{
			preShootPos = prePegPos;							// skip this step
			shootPos    = prePegPos;							// skip this step
		}
		finishPos       = new Vector2( -86,-126);
*/
		
		
		
    
		// use negative x coords because original orientation is assumed 0
		
		startPos        = new Vector2(18f, 48f*yScale);
		prePegPos       = new Vector2(60f, 48f*yScale);
		pegPos          = new Vector2(96f,  0f*yScale);
		postPegPos      = prePegPos;	// in reverse
		// TODO: zero point turn to new orientation
		if (lane==1)
			preShootPos = new Vector2(36f, -24f);		// go around airship
		else
			preShootPos = new Vector2(36f,  0f);
			
		shootPos        = new Vector2( 36f, 48f);
		finishPos       = new Vector2(160f, -48f);
 

		// making everying relative to startPos
		// TODO: allow absolute positioning, not just relative positioning
		prePegPos.sub(startPos);
		postPegPos.sub(startPos);
		preShootPos.sub(startPos);
		shootPos.sub(startPos);
		finishPos.sub(startPos);
		startPos.sub(startPos);
		
		
		
    
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

    public Vector2 getInitialPosition()
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
