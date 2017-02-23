package org.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import org.team686.lib.util.Path;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Path.Waypoint;
import org.team686.lib.util.Vector;
import org.team686.simsbot.Constants;
import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.AutoModeEndedException;
import org.team686.simsbot.auto.actions.*;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class AutoPlacePegMode extends AutoModeBase 
{
	List<Waypoint> path1, path2, path3, path4, path5, path6, path7;
	int lane;
	Pose initialPose = new Pose();
	
	Pose start1 = new Pose(120,-90,180*Pose.degreesToRadians);
	Pose start2 = new Pose(120,  0,180*Pose.degreesToRadians);
	Pose start3 = new Pose(120,+90,180*Pose.degreesToRadians);
	
	Vector approach1 = new Vector( 45,-90);
	Vector approach2 = new Vector( 90,  0);
	Vector approach3 = new Vector( 45,+90);
	
	Vector vision1 = new Vector( 30,-64);
	Vector vision2 = new Vector( 60,  0);
	Vector vision3 = new Vector( 36,+64);
	
	Vector target1 = new Vector(  0,-12);
	Vector target2 = new Vector(  0,  0);
	Vector target3 = new Vector(  0,+12);
	
	
    public AutoPlacePegMode(int _lane, boolean isShooting) 
    {
    	initialPose = new Pose();
    	
    	lane = _lane;
		double vel = 50;

		switch (lane)
		{
		case 1:
		case 2:
		case 3:
		default:
			initialPose = new Pose(start3);
			
			path1 = new ArrayList<>();
			path1.add(new Waypoint(initialPose.getPosition(),    vel));
			path1.add(new Waypoint(approach3, vel));
			path1.add(new Waypoint(  vision3,   vel));
			
			path2 = new ArrayList<>();
			path2.add(new Waypoint(  target3, vel));
			path2.add(new Waypoint(approach3, vel));
			
			path3 = new ArrayList<>();
			path3.add(new Waypoint(approach3, vel));
			path3.add(new Waypoint(approach2, vel));
			path3.add(new Waypoint(  vision2, vel));
			
			path4 = new ArrayList<>();
			path4.add(new Waypoint(  target2, vel));
			path4.add(new Waypoint(approach2, vel));

			path5 = new ArrayList<>();
			path5.add(new Waypoint(approach2, vel));
			path5.add(new Waypoint(approach1, vel));
			path5.add(new Waypoint(  vision1, vel));

			path6 = new ArrayList<>();
			path6.add(new Waypoint(  target1, vel));
			path6.add(new Waypoint(approach1, vel));

			path7 = new ArrayList<>();
			path7.add(new Waypoint(approach1, vel));
			path7.add(new Waypoint(approach2, vel));
			path7.add(new Waypoint(initialPose.getPosition(), vel));
			
		}
		
				
    }

    @Override
    public Pose getInitialPose()
    {
    	return initialPose;
    }
    
    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting AutoPlacePegMode, lane " + lane);

    	// drive towards target
        runAction(new PathFollowerAction(new Path(path1), false));   
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
        
        runAction(new PathFollowerAction(new Path(path2), true));   
        runAction(new PathFollowerAction(new Path(path3), false));   
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
        
        runAction(new PathFollowerAction(new Path(path4), true));   
        runAction(new PathFollowerAction(new Path(path5), false));   
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
        
        runAction(new PathFollowerAction(new Path(path6), true));   
        runAction(new PathFollowerAction(new Path(path7), false));   
    }
    
}
