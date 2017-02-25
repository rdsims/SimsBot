package org.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.List;

import org.team686.lib.util.Path;
import org.team686.lib.util.PathSegment;
import org.team686.lib.util.Pose;
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
public class AutoPlacePegMode extends AutoModeBase 
{
	int lane;
	
	Pose start1 = new Pose(120,-90,180*Pose.degreesToRadians);
	Pose start2 = new Pose(120,  0,180*Pose.degreesToRadians);
	Pose start3 = new Pose(120,+90,180*Pose.degreesToRadians);
	
	Vector2d approach1 = new Vector2d( 45,-90);
	Vector2d approach2 = new Vector2d( 90,  0);
	Vector2d approach3 = new Vector2d( 45,+90);
	
	Vector2d flip3 = new Vector2d( 20,+90);

	Vector2d target1 = new Vector2d(  0,-12);
	Vector2d target2 = new Vector2d(  0,  0);
	Vector2d target3 = new Vector2d(  0,+12);
	
	List<Path> pathList;
	
    public AutoPlacePegMode(int _lane, boolean isShooting) 
    {
    	initialPose = new Pose();
   	
    	lane = _lane;
    	
    	PathSegment.Options pathOptions   = new PathSegment.Options(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	PathSegment.Options visionOptions = new PathSegment.Options(Constants.kVisionMaxVel,        Constants.kVisionMaxAccel,        Constants.kPathFollowingLookahead, true);
    	
		switch (lane)
		{
		case 1:
		case 2:
		case 3:
		default:
			initialPose = new Pose(start3);
System.out.println("initialPose = " + initialPose);
			
			pathList = new ArrayList<Path>();

			// drive to target 1
			Path path1 = new Path();
			path1.add(new Waypoint(initialPose.getPosition(), pathOptions));
			path1.add(new Waypoint(approach3, 				  visionOptions));	// enable vision
			path1.add(new Waypoint(  target3, 				  visionOptions));
			pathList.add(path1);
			
			// back up
			Path path2 = new Path();
			path2.add(new Waypoint(  target3, pathOptions));
			path2.add(new Waypoint(approach3, pathOptions));
			path2.setReverseDirection();
			pathList.add(path2);
			
			// drive to target 2
			Path path3 = new Path();
			path3.add(new Waypoint(approach3, pathOptions));
			path3.add(new Waypoint(approach2, visionOptions));	// enable vision
			path3.add(new Waypoint(  target2, visionOptions));
			pathList.add(path3);

			// back up
			Path path4 = new Path();
			path4.add(new Waypoint(  target2, pathOptions));
			path4.add(new Waypoint(approach2, pathOptions));
			path4.setReverseDirection();
			pathList.add(path4);

			// drive to target 3
			Path path5 = new Path();
			path5.add(new Waypoint(approach2, pathOptions));
			path5.add(new Waypoint(approach1, visionOptions));	// enable vision
			path5.add(new Waypoint(  target1, visionOptions));	
			pathList.add(path5);

			// backup
			Path path6 = new Path();
			path6.add(new Waypoint(  target1, pathOptions));
			path6.add(new Waypoint(approach1, pathOptions));
			path6.setReverseDirection();
			pathList.add(path6);

			// drive back to starting point
			Path path7 = new Path();
			path7.add(new Waypoint(approach1, 	pathOptions));
			path7.add(new Waypoint(approach3, 	pathOptions));
			path7.add(new Waypoint(flip3, 		pathOptions));
			pathList.add(path7);

			// backup to initial position
			Path path8 = new Path();
			path8.add(new Waypoint(flip3, 				  	  pathOptions));
			path8.add(new Waypoint(initialPose.getPosition(), pathOptions));
			path8.setReverseDirection();
			pathList.add(path8);

		}
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting AutoPlacePegMode, lane " + lane);

    	for (int k=0; k<pathList.size(); k++)
            runAction( new PathFollowerWithVisionAction( pathList.get(k) ) );   
    		
    }
    
}
