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
public class SimplePathMode extends AutoModeBase {

    public SimplePathMode(int lane, boolean shouldDriveBack) 
    {
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
    	System.out.println("Starting Auto Mode: Square Pattern");


    	double vel = 48.0;
    	
       	double XX = 48.0;
       	double X = 96.0;
        double Y = 18.0;
    	/* hit Matthew
       	double XX = 36.0;
       	double X = 96.0;
        double Y = 48.0;
    	*/
    	
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(  0, 0), vel));
        first_path.add(new Waypoint(new Translation2d( XX, 0), vel));
        first_path.add(new Waypoint(new Translation2d( XX, Y), vel));
        first_path.add(new Waypoint(new Translation2d(  X, Y), vel));

        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d(  X, Y), vel));
        return_path.add(new Waypoint(new Translation2d( XX, Y), vel));
        return_path.add(new Waypoint(new Translation2d( XX, 0), vel));
        return_path.add(new Waypoint(new Translation2d(  0, 0), vel));
        
        
        runAction(new FollowPathAction(new Path(first_path), false));       		         

        runAction(new FollowPathAction(new Path(return_path), true));       		         
    }
}
