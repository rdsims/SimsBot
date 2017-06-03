package org.team686.simsbot.auto.actions;

import org.team686.lib.util.DataLogger;
import org.team686.simsbot.command_status.DriveState;
import org.team686.simsbot.subsystems.Drive;

/**
 * DriveStraightAction drives the robot straight at a settable angle, distance,
 * and velocity. This action begins by setting the drive controller, and then
 * waits until the distance is reached.
 *
 * @see Action
 * @see Drive
 * @see Rotation2d
 */
public class DriveStraightAction implements Action {

    private double startingDistance;
    private double mWantedDistance;
    private double mVelocity;
    private double mHeadingDeg;
    private Drive mDrive = Drive.getInstance();
    private DriveState driveStatus = DriveState.getInstance();

    public DriveStraightAction(double distance, double velocity) {
        this(distance, velocity, 0);
    }

    public DriveStraightAction(double distance, double velocity, double headingDeg) {
        mWantedDistance = distance;
        mVelocity = velocity;
        mHeadingDeg = headingDeg;
    }

    @Override
    public void start() 
    {
        startingDistance = getCurrentDistance();
        mDrive.setVelocityHeadingSetpoint(mVelocity, mHeadingDeg);
    }

    @Override
    public void update() 
    {
    }

    @Override
    public boolean isFinished() 
    {
System.out.printf("startingDistance=%7.3f, currDist=%7.3f, mWantedDistance=%7.3f\n", startingDistance, getCurrentDistance(), mWantedDistance);    	
    	
        boolean rv = false;
        if (mWantedDistance > 0) 
            rv = (getCurrentDistance() - startingDistance) >= mWantedDistance;
        else
            rv = (getCurrentDistance() - startingDistance) <= mWantedDistance;
        
        return rv;
    }

    @Override
    public void done()
    {
        mDrive.setVelocitySetpoint(0, 0);
    }

    private double getCurrentDistance() 
    {
        return (driveStatus.getLeftDistanceInches() + driveStatus.getRightDistanceInches()) / 2;
    }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("AutoAction", "DriveStraight" );
			put("DriveCmd/talonMode", driveStatus.getTalonControlMode().toString() );
			put("DriveCmd/left", mDrive.getCommand().getLeftMotor() );
			put("DriveCmd/right", mDrive.getCommand().getRightMotor() );
    		put("DriveStatus/TalonControlMode", driveStatus.getTalonControlMode().toString() );
			put("DriveStatus/lSpeed", driveStatus.getLeftSpeedInchesPerSec() );
			put("DriveStatus/rSpeed", driveStatus.getRightSpeedInchesPerSec() );
    		put("DriveStatus/lDistance", driveStatus.getLeftDistanceInches() );
    		put("DriveStatus/rDistance", driveStatus.getRightDistanceInches() );
    		put("DriveStatus/Heading", driveStatus.getHeadingDeg() );
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
