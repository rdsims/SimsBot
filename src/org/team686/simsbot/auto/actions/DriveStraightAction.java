package org.team686.simsbot.auto.actions;

import org.team686.simsbot.DataLogger;
import org.team686.simsbot.subsystems.Drive;
import org.team686.lib.util.DriveStatus;
import org.team686.lib.util.Rotation2d;

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
    private double mHeading;
    private Drive mDrive = Drive.getInstance();
    private DriveStatus driveStatus = DriveStatus.getInstance();

    public DriveStraightAction(double distance, double velocity) {
        this(distance, velocity, 0);
    }

    public DriveStraightAction(double distance, double velocity, double heading) {
        mWantedDistance = distance;
        mVelocity = velocity;
        mHeading = heading;
    }

    @Override
    public void start() {
        startingDistance = getCurrentDistance();
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        boolean rv = false;
        if (mWantedDistance > 0) {
            rv = getCurrentDistance() - startingDistance >= mWantedDistance;
        } else {
            rv = getCurrentDistance() - startingDistance <= mWantedDistance;
        }
        if (rv) {
            mDrive.setVelocitySetpoint(0, 0);
        }
        return rv;
    }

    @Override
    public void done() {
        mDrive.setVelocitySetpoint(0, 0);
    }

    private double getCurrentDistance() {
        return (driveStatus.getLeftDistanceInches() + driveStatus.getRightDistanceInches()) / 2;
    }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
