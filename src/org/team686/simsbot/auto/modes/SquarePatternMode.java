package org.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import org.mini2Dx.gdx.math.Vector2;

import org.team686.lib.util.Path;
import org.team686.lib.util.Path.Waypoint;

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

    	double vel = 36.0;
    	 
    	float D = 72.0f;
    	
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Vector2( 0f, 0f), vel));
        first_path.add(new Waypoint(new Vector2( D, 0f), vel));
        first_path.add(new Waypoint(new Vector2( D, D), vel));
        first_path.add(new Waypoint(new Vector2( 0f, D), vel));
        first_path.add(new Waypoint(new Vector2( 0f, 0f), vel));
        
        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Vector2( 0f, 0f), vel));
        return_path.add(new Waypoint(new Vector2( 0f, D), vel));
        return_path.add(new Waypoint(new Vector2( D, D), vel));
        return_path.add(new Waypoint(new Vector2( D, 0f), vel));
        return_path.add(new Waypoint(new Vector2( 0f, 0f), vel));
        
        runAction(new FollowPathAction(new Path(first_path), false));   
        
        runAction(new FollowPathAction(new Path(return_path), true));       		         
    }
}
