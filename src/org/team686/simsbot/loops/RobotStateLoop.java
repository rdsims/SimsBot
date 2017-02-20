package org.team686.simsbot.loops;

import org.team686.simsbot.RobotState;
import org.team686.lib.util.DriveStatus;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateLoop implements Loop 
{
    static RobotStateLoop instance = new RobotStateLoop();
    public static RobotStateLoop getInstance() { return instance; }

    RobotState robotState;
    DriveStatus driveStatus;
    
    RobotStateLoop() 
    {
        robotState = RobotState.getInstance();
        driveStatus = DriveStatus.getInstance();
    }
    


    @Override
    public void onStart() 
    {
    	robotState.setPrevEncoderDistance(driveStatus.getLeftDistanceInches(), driveStatus.getRightDistanceInches());
    }

    @Override
    public void onLoop() 
    {
    	// the following driveStatus elements are set during DriveLoop, called just previous to RobotStateLoop,
    	// and in the same LoopController thread
    	
        double time      	= Timer.getFPGATimestamp();
        double lDistance 	= driveStatus.getLeftDistanceInches();
        double rDistance 	= driveStatus.getRightDistanceInches();
        double lSpeed    	= driveStatus.getLeftSpeedInchesPerSec();
        double rSpeed    	= driveStatus.getRightSpeedInchesPerSec(); 
        double gyroAngleRad = driveStatus.getHeadingRad();

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngleRad);
    }

    @Override
    public void onStop() 
    {
        // no-op
    }

}
