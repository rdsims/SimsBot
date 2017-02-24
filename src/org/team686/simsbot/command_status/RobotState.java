package org.team686.simsbot.command_status;

import org.team686.simsbot.command_status.DriveStatus;
import org.team686.lib.util.DataLogger;
import org.team686.lib.util.InterpolatingDouble;
import org.team686.lib.util.InterpolatingTreeMap;
import org.team686.lib.util.Kinematics;
import org.team686.lib.util.Pose;

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

    private double gyroCorrection;
    
    private double prevLeftDistance = 0;
    private double prevRightDistance = 0;
    
    protected RobotState() { reset(0, new Pose()); }

	public synchronized void reset(double start_time, Pose initialFieldToRobot) 
	{
		// calibrate initial position to initial pose (set by autonomous mode)
        fieldToRobot = new InterpolatingTreeMap<>(kObservationBufferSize);
        fieldToRobot.put(new InterpolatingDouble(start_time), initialFieldToRobot);
        
        // calibrate initial heading to initial pose (set by autonomous mode)
        double desiredHeading = initialFieldToRobot.getHeadingRad();  
        double gyroHeading  = DriveStatus.getInstance().getHeadingRad();
        gyroCorrection = gyroHeading - desiredHeading;		// subtract gyroCorrection from actual gyro heading to get desired orientation
        
        robotSpeed = new Pose.Delta(0, 0);
        setPrevEncoderDistance(0, 0);
    }
	
	public void setPrevEncoderDistance(double _prevLeftDistance, double _prevRightDistance)
	{
        prevLeftDistance  = _prevLeftDistance;
        prevRightDistance = _prevRightDistance;     
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
    	Pose.Delta delta = new Pose.Delta(robotSpeed.dDistance * lookahead_time, robotSpeed.dHeading * lookahead_time);
        return Kinematics.travelArc(getLatestFieldToVehicle(), delta);
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose observation)
    {
        fieldToRobot.put(new InterpolatingDouble(timestamp), observation);
    }

    public void generateOdometryFromSensors(double _time, double _lEncoderDistance, double _rEncoderDistance, 
    		                                double _lEncoderSpeed, double _rEncoderSpeed, double _gyroAngleRad) 
    {
        Pose lastPose = getLatestFieldToVehicle();
        
        // get change in encoder distance from last call
        double dLeftDistance  = _lEncoderDistance - prevLeftDistance; 
        double dRightDistance = _rEncoderDistance - prevRightDistance;

        setPrevEncoderDistance(_lEncoderDistance, _rEncoderDistance);
                
        Pose       odometry = Kinematics.integrateForwardKinematics(lastPose, dLeftDistance, dRightDistance, _gyroAngleRad - gyroCorrection);
        Pose.Delta velocity = Kinematics.forwardKinematics(_lEncoderSpeed, _rEncoderSpeed);
        
        addFieldToVehicleObservation(_time, odometry);	// store odometry
        robotSpeed = velocity;							// used in getPredictedFieldToVehicle()
    }

    public double getSpeed()
    {
    	return robotSpeed.dDistance;
    }
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            Pose odometry = getLatestFieldToVehicle();
            put("RobotState/positionX",  odometry.getX());
            put("RobotState/positionY",  odometry.getY());
            put("RobotState/headingDeg", odometry.getHeadingDeg());
        }
    };
    
    public DataLogger getLogger() { return logger; }
    
}
