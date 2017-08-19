package org.team686.lib.util;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

import java.util.Optional;

import org.team686.lib.util.Kinematics.WheelSpeed;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.command_status.GoalStates;
import org.team686.simsbot.command_status.GoalStates.GoalState;
import org.team686.simsbot.subsystems.Drive;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class PathFollowerWithVisionDriveController 
{ 
	/* Controller can be started in PATH_FOLLOWING or VISION state, as set by _initialState in constructor
	 * If in PATH_FOLLOWING state, it will transition to VISION state when the robot locates a vision target.  
	 * It will only search for vision targets on vision-enabled path segments (set in PathSegmentOptions)
	 * It will not transition back to PATH_FOLLOWING.
	 * Start a new Action to continue following a path moving after this Action has completed.
	 */
	public enum PathVisionState { PATH_FOLLOWING, VISION };
	
	public PathVisionState state;
	boolean reversed;	// always false when vision is enabled.  kept so that path follower code can still be used without vision
	
	public Drive drive = Drive.getInstance();
	public RobotState robotState = RobotState.getInstance();
	public GoalStates goalStates = GoalStates.getInstance();
	private Relay ledRelay = LedRelay.getInstance();
	
	Path path;
	
	public Vector2d avgTargetLocation = new Vector2d(0,0);

	public double distanceFromPath;
	public double lookaheadDist;
	public Vector2d lookaheadPoint = new Vector2d();
	public double bearingToTarget;		
	
	public double currentTime;

	public Pose currentPose = new Pose();
	public Pose previousPose = new Pose();
	
	// camera pose with respect to robot
	public Pose cameraPose_Robot = new Pose(Constants.kCameraPoseX, Constants.kCameraPoseY, Constants.kCameraPoseThetaRad);
	
	public double prevDistanceToTargetInches;
	public double prevHeadingToTarget;
	
	public Vector2d targetLocation = new Vector2d(0,0);
	public double distanceToTargetInches;

	public double remainingDistance;

	public double speed;
	public double curvature;

	public WheelSpeed wheelSpeed;
	
	private double prevSpeed;
	private double prevTime;

	
    public PathFollowerWithVisionDriveController(Path _path, PathVisionState _initialState) 
    {
        drive = Drive.getInstance();
        path = _path;
        state = _initialState;
    }

    public void start() 
    {
		prevSpeed = 0;
		prevTime  = -1;		
        remainingDistance = Double.MAX_VALUE;	// make sure we run update() at least once before finishing
    }


    public void update() 
    {
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		currentPose = robotState.getLatestFieldToVehicle();		
		currentTime = Timer.getFPGATimestamp();

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		wheelSpeed = pathVisionDrive(currentTime, currentPose);		// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
        drive.setVelocitySetpoint(wheelSpeed);
	}

	boolean visionEnabledPrevSegment = false;

    
	public WheelSpeed pathVisionDrive(double _currentTime, Pose _currentPose)
	{
		if (prevTime < 0)				// initial setting of prevTime is important to limit initial acceleration
			prevTime = _currentTime;	// avoid calling Timer.getFPGATimestamp() in this function to allow off-robot testing
		
		
		remainingDistance = Double.MAX_VALUE;
		double maxSpeed = 0;
		double maxAccel = 0;
		
		reversed = path.getReverseDirection();
		
		boolean visionEnabledSegment = path.getSegmentVisionEnable();
		
		// turn on/off LEDs when changing to/from vision-enabled segments
		if (visionEnabledSegment && !visionEnabledPrevSegment)
			ledRelay.set(Relay.Value.kOn);
		else if (!visionEnabledSegment && visionEnabledPrevSegment)
			ledRelay.set(Relay.Value.kOff);
			
		if (visionEnabledSegment)
			visionDrive(_currentTime, _currentPose);
		else
			pathDrive(_currentTime, _currentPose);

		boolean visionEnabledPrevSegment = visionEnabledSegment; 
		
		if (state == PathVisionState.PATH_FOLLOWING)	 
		{
			remainingDistance = path.getRemainingLength();		// TODO: address stopping when past final segment
			maxSpeed = path.getSegmentMaxSpeed();
			maxAccel = path.getSegmentMaxAccel();
		}
		else
		{
			remainingDistance = distanceToTargetInches - Constants.kPegTargetDistanceThresholdFromCameraInches;
			maxSpeed = Constants.kVisionMaxVel;
			maxAccel = Constants.kVisionMaxAccel;
		}
		
		speedControl(_currentTime, remainingDistance, maxSpeed, maxAccel);

		if (reversed)
		{
			speed = -speed;
			curvature = -curvature;	// TODO: simplify by removing this, and removing flipping heading 180 degrees below?
		}
		
		wheelSpeed = Kinematics.inverseKinematicsFromSpeedCurvature(speed, curvature);
		wheelSpeed.limit(maxSpeed);
		return wheelSpeed;
	}
	
	public WheelSpeed getWheelVelocity() { return getWheelVelocity(); }
	public Path   getPath() { return path; }	// warning: not returning a defensive copy
	public double getDistanceFromPath() { return distanceFromPath; }
	public PathVisionState getPathVisionState() { return state; }

	
	// Drive towards lookahead point on path
	private void pathDrive(double _currentTime, Pose _currentPose)
	{
		state = PathVisionState.PATH_FOLLOWING;
				
		//---------------------------------------------------
		// Find Lookahead Point
		//---------------------------------------------------
		distanceFromPath = path.update(_currentPose.getPosition());
		lookaheadPoint = path.getLookaheadPoint(_currentPose.getPosition(), distanceFromPath);
		
		//---------------------------------------------------
		// Find arc to travel to Lookahead Point
		//---------------------------------------------------
		Vector2d robotToTarget = lookaheadPoint.sub(_currentPose.getPosition());
		double lookaheadDist = robotToTarget.length();
		bearingToTarget = robotToTarget.angle() - _currentPose.getHeading();
		if (reversed)
			bearingToTarget -= Math.PI;	// flip robot around
		
		curvature = 2 * Math.sin(bearingToTarget) / lookaheadDist;
	}

	
	
	// drive towards vision target (or follow path if no target acquired)
	public void visionDrive(double _currentTime, Pose _currentPose)
	{
		Optional<GoalState> optGoalState = goalStates.getBestVisionTarget();

		// If we get a valid message from the Vision co-processor, update our estimate of the target location
		if (optGoalState.isPresent()) 
		{
			state = PathVisionState.VISION;
			
			// Get range and angle to target
			GoalState goalState = optGoalState.get();
			distanceToTargetInches = goalState.getHorizontalDistance();
			bearingToTarget = goalState.getRelativeBearing();
			
			// Calculate motor settings to turn towards target   
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);	// length of chord <= kVisionLookaheadDist
			curvature     = 2 * Math.sin(bearingToTarget) / lookaheadDist;						// curvature = 1/radius of circle (positive: turn left, negative: turn right)
		}
		else
		{
			// target not acquired -- speed/curvature will be controlled by path follower
			pathDrive(_currentTime, _currentPose);
		}
	}
	
	
	
	// keep speed within acceleration limits
	public void speedControl(double _currentTime, double _remainingDistance, double _maxSpeed, double _maxAccel)
	{
		//---------------------------------------------------
		// Apply speed control
		//---------------------------------------------------
		speed = _maxSpeed;
		
		double dt = _currentTime - prevTime;
		
		// apply acceleration limits
		double accel = (speed - prevSpeed) / dt;
		if (accel > _maxAccel)
			speed = prevSpeed + _maxAccel * dt;
		else if (accel < -_maxAccel)
			speed = prevSpeed - _maxAccel * dt;

		// apply braking distance limits
		// vf^2 = v^2 + 2*a*d   Solve for v, given vf=0, configured a, and measured d
		double stoppingDistance = _remainingDistance;
		double maxBrakingSpeed = Math.sqrt(2.0 * _maxAccel * stoppingDistance);
		if (Math.abs(speed) > maxBrakingSpeed)
			speed = Math.signum(speed) * maxBrakingSpeed;

		// apply minimum velocity limit (Talons can't track low speeds well)
		final double kMinSpeed = 4.0;
		if (Math.abs(speed) < kMinSpeed) 
			speed = Math.signum(speed) * kMinSpeed;

		// store for next time through loop	
		prevTime = _currentTime;
		prevSpeed = speed;
	}
		
	
    public boolean isFinished() 
    {
    	boolean done = false;
    	
    	if (state == PathVisionState.PATH_FOLLOWING)
	        done = (remainingDistance <= Constants.kPathFollowingCompletionTolerance);
    	else
    		done = (remainingDistance <= Constants.kVisionCompletionTolerance);
    	
    	return done;
    }

    public void done() 
    {
		// cleanup code, if any
    	ledRelay.set(Relay.Value.kOff); 		// turn off LEDs when done
    	drive.stop();
    }

 
    
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {            
			DriveCommand cmd = drive.getCommand();
    		put("PathVision/driveMode", cmd.getDriveControlMode().toString() );
    		put("PathVision/talonMode", cmd.getTalonControlMode().toString() );
    		put("PathVision/left",  cmd.getLeftMotor() );
       		put("PathVision/right", cmd.getRightMotor() );
       		put("PathVision/brake", cmd.getBrake() );
       		
       		Pose odometry = robotState.getLatestFieldToVehicle();
            put("PathVision/positionX",  odometry.getX());
            put("PathVision/positionY",  odometry.getY());
            put("PathVision/headingDeg", odometry.getHeadingDeg());
        	
			put("PathVision/reversed", reversed);
			put("PathVision/state", state.toString());

			put("PathVision/segmentStartX", path.getSegmentStart().getX());
			put("PathVision/segmentStartY", path.getSegmentStart().getY());
			put("PathVision/segmentEndX", path.getSegmentEnd().getX());
			put("PathVision/segmentEndY", path.getSegmentEnd().getY());
			put("PathVision/segmentMaxSpeed", path.getSegmentMaxSpeed());
			put("PathVision/segmentVisionEnable", path.getSegmentVisionEnable());
			
			put("PathVision/distanceFromPath", distanceFromPath );
			put("PathVision/lookaheadDist", lookaheadDist );
			put("PathVision/lookaheadPointX",  lookaheadPoint.getX() );
			put("PathVision/lookaheadPointY",  lookaheadPoint.getY());

			put("PathVision/prevPoseX", previousPose.getX());
			put("PathVision/prevPoseY", previousPose.getY());
						
			put("PathVision/remainingDistance",  remainingDistance );
			
			put("PathVision/speed", 	speed);
			put("PathVision/curvature", curvature );
			put("PathVision/lSpeed", 	wheelSpeed.left);
			put("PathVision/rSpeed", 	wheelSpeed.right);
			
        }
    };
	
    public DataLogger getLogger() { return logger; }
}
