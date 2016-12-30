package org.usfirst.frc.team686.simsbot.loops;

import org.usfirst.frc.team686.simsbot.Kinematics;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.lib.util.RigidTransform2d;
import org.usfirst.frc.team686.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

/**
 * Periodically estimates the state of the robot using the robot's distance
 * traveled (compares two waypoints), gyroscope orientation, and velocity, among
 * various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    RobotState robot_state_ = RobotState.getInstance();
    Drive drive_ = Drive.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;

    @Override
    public void onStart() {
        left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = drive_.getRightDistanceInches();
    }

    @Override
    public void onLoop() {
        double time = Timer.getFPGATimestamp();
        double left_distance  = drive_.getLeftDistanceInches();
        double right_distance = drive_.getRightDistanceInches();
        double left_speed  = drive_.getLeftVelocityInchesPerSec();
        double right_speed = drive_.getRightVelocityInchesPerSec(); 
        Rotation2d gyro_angle = Rotation2d.fromDegrees(drive_.getHeading());

        RigidTransform2d odometry = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        RigidTransform2d.Delta velocity = Kinematics.forwardKinematics(left_speed, right_speed);
                
        robot_state_.addObservations(time, odometry, velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    @Override
    public void onStop() {
        // no-op
    }

}
