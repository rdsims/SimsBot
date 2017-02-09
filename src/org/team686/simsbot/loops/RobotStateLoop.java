package org.team686.simsbot.loops;

import org.team686.simsbot.Kinematics;
import org.team686.simsbot.RobotState;
import org.team686.simsbot.subsystems.Drive;
import org.team686.lib.util.DriveStatus;
import org.team686.lib.util.RigidTransform2d;
import org.team686.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateLoop implements Loop 
{
    static RobotStateLoop instance = new RobotStateLoop();

    public static RobotStateLoop getInstance() 
    {
        return instance;
    }

    RobotStateLoop() {}

    RobotState robotState = RobotState.getInstance();
    DriveStatus driveStatus = Drive.getInstance().driveStatus;
    double prevLeftDistance = 0;
    double prevRightDistance = 0;

    @Override
    public void onStart() 
    {
        prevLeftDistance  = driveStatus.getLeftDistanceInches();
        prevRightDistance = driveStatus.getRightDistanceInches();
    }

    @Override
    public void onLoop() 
    {
        double time = Timer.getFPGATimestamp();
        double leftDistance  = driveStatus.getLeftDistanceInches();
        double rightDistance = driveStatus.getRightDistanceInches();
        double leftSpeed     = driveStatus.getLeftSpeedInchesPerSec();
        double rightSpeed    = driveStatus.getRightSpeedInchesPerSec(); 
        Rotation2d gyroAngle = Rotation2d.fromDegrees(driveStatus.getHeadingDeg());

        RigidTransform2d odometry = robotState.generateOdometryFromSensors(
                leftDistance-prevLeftDistance, rightDistance-prevRightDistance, gyroAngle);
        RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(leftSpeed, rightSpeed);
                
        robotState.addObservations(time, odometry, velocity);
        prevLeftDistance = leftDistance;
        prevRightDistance = rightDistance;
    }

    @Override
    public void onStop() 
    {
        // no-op
    }

}
