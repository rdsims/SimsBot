package org.team686.simsbot.auto.modes;

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
public class SimplePathMode extends AutoModeBase {

    public SimplePathMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");
  	
    	double XX = 48.0;
    	double X = 96.0;
    	double Y = 18.0;
    	
    	PathSegment.Options options = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);

    	
        Path path = new Path();
        path.add(new Waypoint(new Vector2d(  0, 0), options));
        path.add(new Waypoint(new Vector2d( XX, 0), options));
        path.add(new Waypoint(new Vector2d( XX, Y), options));
        path.add(new Waypoint(new Vector2d(  X, Y), options));

        Path revPath = new Path(path);
        revPath.setReverseOrder();
        revPath.setReverseDirection();
        
        runAction(new PathFollowerWithVisionAction(path));			// drive forward
        runAction(new PathFollowerWithVisionAction(revPath));    	// drive reversed 
    }
}
