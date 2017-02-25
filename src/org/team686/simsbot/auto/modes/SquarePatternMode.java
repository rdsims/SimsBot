package org.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import org.team686.lib.util.Path;
import org.team686.lib.util.PathSegment;
import org.team686.lib.util.Path.Waypoint;
import org.team686.lib.util.Vector2d;
import org.team686.simsbot.Constants;
import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.AutoModeEndedException;
import org.team686.simsbot.auto.actions.*;


/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class SquarePatternMode extends AutoModeBase {

    public SquarePatternMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");

    	float D = 72.0f;
    	
    	PathSegment.Options options = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Vector2d( 0, 0), options));
        path.add(new Waypoint(new Vector2d( D, 0), options));
        path.add(new Waypoint(new Vector2d( D, D), options));
        path.add(new Waypoint(new Vector2d( 0, D), options));
        path.add(new Waypoint(new Vector2d( 0, 0), options));
        
        runAction(new PathFollowerWithVisionAction(new Path(path, false)));	// forward   
        runAction(new PathFollowerWithVisionAction(new Path(path, true)));	// reverse       		         
    }
}
