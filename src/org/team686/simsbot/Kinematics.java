package org.team686.simsbot;

import org.team686.lib.util.Pose;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the
 * wheelbase as a differential drive (with a corrective factor to account for
 * the inherent skidding of the center 4 wheels quasi-kinematically).
 */

public class Kinematics 
{
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less
     * accurate than below, but useful for predicting motion)
     */

    public static Pose.Delta forwardKinematics(double ldeltaDist, double rDeltaDist)
    {
    	double deltaDist = (ldeltaDist + rDeltaDist)/2;		// linear speed of center of robot is the average of the left and right
    	double diffDist  = (rDeltaDist - ldeltaDist)/2;		// differential speed of wheels (positive: turning to left, increasing theta)
    	double deltaHeading = diffDist * 2 * Constants.kTrackScrubFactor / Constants.kTrackEffectiveDiameter;		// change in heading due to differential speed
        return new Pose.Delta(deltaDist, deltaHeading);			// change in pose
    }
    
    /**
     * Forward kinematics using encoders and explicitly measured rotation (ie. from gyro)
     */

    public static Pose.Delta forwardKinematics(double lSpeed, double rSpeed, double deltaHeadingRad)
    {
        return new Pose.Delta((lSpeed + rSpeed)/2, deltaHeadingRad);
    }

    
    /** Append the result of forward kinematics to a previous pose. */

    public static Pose integrateForwardKinematics(Pose currentPose, double lSpeed, double rSpeed, double gyroAngleRad)
    {
    	Pose.Delta delta = forwardKinematics(lSpeed, rSpeed, gyroAngleRad - currentPose.getHeadingRad());
        return currentPose.travelArc(delta);
    }
    
    public static class DriveVelocity 
    {
        public double left;
        public double right;

        public DriveVelocity(double left, double right) 
        {
            this.left = left;
            this.right = right;
        }
        
        public void scale(double scale)
        {
            left  *= scale;
            right *= scale;
        }
    }

    /*
     * Calculate left/right wheel speeds that will give desired robot speed and change of heading
     */
    public static DriveVelocity inverseKinematics(Pose.Delta delta) 
    {
    	double diffSpeed = 0.0;
        if (Math.abs(delta.dHeading) > kEpsilon)
        	diffSpeed = delta.dHeading * Constants.kTrackEffectiveDiameter / (2 * Constants.kTrackScrubFactor);

        return new DriveVelocity(delta.dDistance - diffSpeed, delta.dDistance + diffSpeed); 
    }
}
