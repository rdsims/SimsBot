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
import org.team686.simsbot.command_status.RobotState;



/**
 * Go over the defenses in the starting configuration, then launch one ball (in
 * the robot at start)
 */
public class AutoPlacePegMode extends AutoModeBase 
{
	List<Waypoint> path1, path2, path3, path4, path5, path6, path7;
	int lane;
	RobotState robotState;
	
	
	Pose start1 = new Pose(120,-90,180*Pose.degreesToRadians);
	Pose start2 = new Pose(120,  0,180*Pose.degreesToRadians);
	Pose start3 = new Pose(120,+90,180*Pose.degreesToRadians);
	
	Vector2d approach1 = new Vector2d( 45,-90);
	Vector2d approach2 = new Vector2d( 90,  0);
	Vector2d approach3 = new Vector2d( 45,+90);
	
	Vector2d target1 = new Vector2d(  0,-12);
	Vector2d target2 = new Vector2d(  0,  0);
	Vector2d target3 = new Vector2d(  0,+12);
	
	
    public AutoPlacePegMode(int _lane, boolean isShooting) 
    {
    	robotState = RobotState.getInstance();
    	Pose startPose = new Pose();
    	
    	lane = _lane;
    	
		double speed = 50;
    	double lookaheadDist = Constants.kPathFollowingLookahead;
    	
    	PathSegment.PathSegmentOptions path_options   = new PathSegment.PathSegmentOptions(speed, lookaheadDist, false);
    	PathSegment.PathSegmentOptions vision_options = new PathSegment.PathSegmentOptions(speed, lookaheadDist, true);
    	
		switch (lane)
		{
		case 3:
			startPose = start3;
			
			// drive to target 1
			path1 = new ArrayList<>();
			path1.add(new Waypoint(startPose.getPosition(), path_options));
			path1.add(new Waypoint(approach3, 				path_options));
			path2.add(new Waypoint(  target3, 				vision_options));	// enable vision
			
			// back up
			path2 = new ArrayList<>();
			path2.add(new Waypoint(  target3, path_options));
			path2.add(new Waypoint(approach3, path_options));
			
			// drive to target 2
			path3 = new ArrayList<>();
			path3.add(new Waypoint(approach3, path_options));
			path3.add(new Waypoint(approach2, path_options));
			path3.add(new Waypoint(  target2, vision_options));	// enable vision

			// back up
			path4 = new ArrayList<>();
			path4.add(new Waypoint(  target2, path_options));
			path4.add(new Waypoint(approach2, path_options));

			// drive to target 3
			path5 = new ArrayList<>();
			path5.add(new Waypoint(approach2, path_options));
			path5.add(new Waypoint(approach1, path_options));
			path5.add(new Waypoint(  target1, vision_options));	// enable vision

			// backup
			path6 = new ArrayList<>();
			path6.add(new Waypoint(  target1, path_options));
			path6.add(new Waypoint(approach1, path_options));

			// drive back to starting point
			path7 = new ArrayList<>();
			path7.add(new Waypoint(approach1, 				path_options));
			path7.add(new Waypoint(approach2, 				path_options));
			path7.add(new Waypoint(startPose.getPosition(), path_options));
			
		}
		
		System.out.println("InitialPose: " + startPose);
		robotState.reset(0.0, startPose);			
				
    }

     @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting AutoPlacePegMode, lane " + lane);

    	// drive towards target
        runAction(new PathFollowerAction(new Path(path1, false)));   
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
        
        runAction(new PathFollowerAction(new Path(path2, true)));   
        runAction(new PathFollowerAction(new Path(path3, false)));   
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
        
        runAction(new PathFollowerAction(new Path(path4, true)));   
        runAction(new PathFollowerAction(new Path(path5, false)));   
        runAction(new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel));
        
        runAction(new PathFollowerAction(new Path(path6, true)));   
        runAction(new PathFollowerAction(new Path(path7, false)));   
    }
    
}
