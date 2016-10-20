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
    	
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d( 0.0,  0.0), 1.0));
        first_path.add(new Waypoint(new Translation2d( 0.0, 48.0), 1.0));
        first_path.add(new Waypoint(new Translation2d(48.0, 48.0), 1.0));
        first_path.add(new Waypoint(new Translation2d(48.0,  0.0), 1.0));
        first_path.add(new Waypoint(new Translation2d( 0.0,  0.0), 1.0));
        
        runAction(new FollowPathAction(new Path(first_path), false));       		         
    }
}
