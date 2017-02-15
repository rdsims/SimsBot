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
public class SimplePathMode extends AutoModeBase {

    public SimplePathMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");


    	double vel = 48.0f;
    	
    	float XX = 48.0f;
    	float X = 96.0f;
    	float Y = 18.0f;
    	
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Vector2(  0f, 0f), vel));
        first_path.add(new Waypoint(new Vector2( XX, 0f), vel));
        first_path.add(new Waypoint(new Vector2( XX, Y), vel));
        first_path.add(new Waypoint(new Vector2(  X, Y), vel));

        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Vector2(  X, Y), vel));
        return_path.add(new Waypoint(new Vector2( XX, Y), vel));
        return_path.add(new Waypoint(new Vector2( XX, 0f), vel));
        return_path.add(new Waypoint(new Vector2(  0f, 0f), vel));
        
        
        runAction(new FollowPathAction(new Path(first_path), false));       		         

        runAction(new FollowPathAction(new Path(return_path), true));       		         
    }
}
