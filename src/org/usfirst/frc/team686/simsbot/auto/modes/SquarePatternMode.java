package org.usfirst.frc.team686.simsbot.auto.modes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.usfirst.frc.team686.lib.util.Path;
import org.usfirst.frc.team686.lib.util.Translation2d;
import org.usfirst.frc.team686.lib.util.Path.Waypoint;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive.DriveControlState;

import org.usfirst.frc.team686.simsbot.subsystems.Drive;


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
    	 
    	double D = 72.0;
    	
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d( 0, 0), vel));
        first_path.add(new Waypoint(new Translation2d( D, 0), vel));
        first_path.add(new Waypoint(new Translation2d( D, D), vel));
        first_path.add(new Waypoint(new Translation2d( 0, D), vel));
        first_path.add(new Waypoint(new Translation2d( 0, 0), vel));
        
        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d( 0, 0), vel));
        return_path.add(new Waypoint(new Translation2d( 0, D), vel));
        return_path.add(new Waypoint(new Translation2d( D, D), vel));
        return_path.add(new Waypoint(new Translation2d( D, 0), vel));
        return_path.add(new Waypoint(new Translation2d( 0, 0), vel));
        
        runAction(new FollowPathAction(new Path(first_path), false));   
        
        runAction(new FollowPathAction(new Path(return_path), true));       		         
    }
}
