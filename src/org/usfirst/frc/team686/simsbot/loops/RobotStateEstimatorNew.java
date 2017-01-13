package org.usfirst.frc.team686.simsbot.loops;

import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team686.lib.util.RobotPose;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;


/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimatorNew implements Loop
{
    RobotState robotState = RobotState.getInstance();
    Drive drive = Drive.getInstance();

    RobotPose pose;
    
    // store state history
    double lDistPrev = 0;
    double rDistPrev = 0;
    double thetaPrev = 0;

    // small floating point number below which we will use estimate to avoid divide by zero errors
    private final static double kEps = 1E-9;

    static RobotStateEstimatorNew instance = new RobotStateEstimatorNew(0,0,0);

    public static RobotStateEstimatorNew getInstance()
    {
        return instance;
    }

    RobotStateEstimatorNew(double x, double y, double theta)
    {
    	setPose(x,y,theta);
    }
    
    public void setPose(double x, double y, double theta)
    {
    	pose.set(x,y,theta);
    }
    
    @Override
    public void onStart() {
        lDistPrev = drive.getLeftDistanceInches();
        rDistPrev = drive.getRightDistanceInches();
        thetaPrev = drive.getHeading();
    }

    @Override
    public void onLoop() {
        double lDist = drive.getLeftDistanceInches();
        double rDist = drive.getRightDistanceInches();
        double theta = drive.getHeading();
        
        // dD = distance traveled by center of robot in time delta_t
        // dTheta = change in heading over last delta_t
        // thetaMid = midpoint of initial and final headings
        double dD = ((lDist-lDistPrev) + (rDist-rDistPrev) / 2.0);	
        double dTheta = (theta - thetaPrev);
        double thetaMid = (theta + thetaPrev) / 2.0;
        
        // assume robot traveled in a circular arc over previous delta_t
        // the initial and final heading define the starting and ending
        // points on the circle
        // calculate length of chord connecting starting and ending points
        double chordLength;
        if (Math.abs(dTheta) > kEps)
        {
        	double arcLength = dD / dTheta;
        	chordLength = 2*Math.sin(dTheta/2.0)*arcLength;
        }
        else
        {
        	// heading didn't change, travel was straight
        	// avoid divide by zero error
        	// sin(theta/2)/theta ~ 1/2 for theta near zero
        	chordLength = dD;
        }

        // update heading
        pose.x += chordLength * Math.cos(thetaMid);
        pose.y += chordLength * Math.sin(thetaMid);
        pose.theta = theta;
        
        
        // store sensor state for next iteration
        lDistPrev = lDist;
        rDistPrev = rDist;
        thetaPrev = theta;

        // logging to account for vision latency
        //double time = Timer.getFPGATimestamp();
        //double lSpeed = drive.getLeftVelocityInchesPerSec();
        //double rSpeed = drive.getRightVelocityInchesPerSec();
    
    }

    @Override
    public void onStop()
    {
		// nothing to do
	}

}
