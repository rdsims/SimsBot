package org.team686.simsbot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import org.team686.lib.util.AdaptivePurePursuitController;
import org.team686.lib.util.InterpolatingDouble;
import org.team686.lib.util.InterpolatingTreeMap;
import org.team686.lib.util.Pose;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotState keeps track of the poses of various coordinate frames throughout
 * the match. A coordinate frame is simply a point and direction in space that
 * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
 * spatial relationship between different frames.
 * 
 * Robot frames of interest (from parent to child):
 * 
 * 1. Field frame: origin is where the robot is turned on
 * 
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 * 
 * 3. Turret fixed frame: origin is the center of the turret when the turret is
 * at 0 degrees rotation relative to the vehicle frame
 * 
 * 4. Turret rotating frame: origin is the center of the turret as it rotates
 * 
 * 5. Camera frame: origin is the center of the camera imager as it rotates with
 * the turret
 * 
 * 6. Goal frame: origin is the center of the goal (note that orientation in
 * this frame is arbitrary). Also note that there can be multiple goal frames.
 * 
 * As a kinematic chain with 6 frames, there are 5 transforms of interest:
 * 
 * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably drift, but is usually accurate over
 * short time periods.
 * 
 * 2. Vehicle-to-turret-fixed: This is a constant.
 * 
 * 3. Vehicle-to-turret-rotating: This is a pure rotation, and is tracked over
 * time using the turret encoder.
 * 
 * 4. Turret-rotating-to-camera: This is a constant.
 * 
 * 5. Camera-to-goal: This is a pure translation, and is measured by the vision
 * system.
 */

public class RobotState 
{
    private static RobotState instance = new RobotState();
    public static RobotState getInstance() { return instance; }

    public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;

    protected InterpolatingTreeMap<InterpolatingDouble, Pose> fieldToRobot;
    protected Pose.Delta robotSpeed;

    protected RobotState() { reset(0, new Pose()); }

	public synchronized void reset(double start_time, Pose initialFieldToRobot) 
	{
        fieldToRobot = new InterpolatingTreeMap<>(kObservationBufferSize);
        fieldToRobot.put(new InterpolatingDouble(start_time), initialFieldToRobot);
        robotSpeed = new Pose.Delta(0, 0);
    }

	public synchronized Pose getFieldToVehicle(double timestamp) 
	{
        return fieldToRobot.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose getLatestFieldToVehicle() 
    {
        return fieldToRobot.lastEntry().getValue();
    }

    public synchronized Pose getPredictedFieldToVehicle(double lookahead_time) 
    {
    	Pose.Delta delta = new Pose.Delta(robotSpeed.dDistance * lookahead_time, robotSpeed.dHeadingRad * lookahead_time);
        return getLatestFieldToVehicle().travelArc(delta);
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose observation)
    {
        fieldToRobot.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose field_to_vehicle, Pose.Delta velocity) 
    {
        addFieldToVehicleObservation(timestamp, field_to_vehicle);
        robotSpeed = velocity;
    }

    public Pose generateOdometryFromSensors(double lEncoderDeltaDistance, double rEncoderDeltaDistance, double currentGyroAngle) 
    {
        Pose lastPose = getLatestFieldToVehicle();
        return Kinematics.integrateForwardKinematics(lastPose, lEncoderDeltaDistance, rEncoderDeltaDistance, currentGyroAngle);
    }

    
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            Pose odometry = getLatestFieldToVehicle();
            put("RobotState/X", odometry.getX());
            put("RobotState/Y", odometry.getY());
            put("RobotState/H", odometry.getHeadingDeg());
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
}
