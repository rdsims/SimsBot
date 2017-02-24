package org.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.command_status.VisionStatus;
import org.team686.simsbot.subsystems.Drive;
import org.team686.lib.util.DataLogger;
import org.team686.lib.util.Path;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class PathFollowerWithVisionAction implements Action 
{
	Path path;
	boolean reversed;	// always false when vision is enabled.  kept so that path follower code can still be used without vision
	
	public Drive drive = Drive.getInstance();
	public RobotState robotState = RobotState.getInstance();
	public VisionStatus visionStatus = VisionStatus.getInstance();

	private boolean hasStarted;
	
	double nominalLookaheadDist;
	double nominal_dt;
	double pathCompletionTolerance;

	// properties needed for class
	public boolean targetAcquired; 
	public Vector2d avgTargetLocation = new Vector2d(0,0);
	public int  avgCnt;

	public double distanceFromPath;
	public double lookaheadDist;
	public Vector2d lookaheadPoint;
	public double headingToTargetRadians;
	public double remainingDistance;
		
	
	// for logging only
	public double currentTime;
	public double imageTimestamp;
	public double normalizedTargetX;
	public double normalizedTargetWidth;
	public Pose previousPose = new Pose();
	public double prevDistanceToTargetInches;
	public double prevHeadingToTargetRadians;
	public Vector2d targetLocation = new Vector2d(0,0);
	public Pose currentPose = new Pose();
	public double distanceToTargetInches;
	public double curvature;
	public double speed;

	private double prevSpeed;
	private double prevTime;

	
    public PathFollowerWithVisionAction(Path _path) 
    {
        path = _path;

        drive = Drive.getInstance();
		nominal_dt = Constants.kLoopDt;
		pathCompletionTolerance = 1.0;
    }

    @Override
    public void start() 
    {
		System.out.println("Starting PathFollowerWithVisionAction");
		prevSpeed = 0;
		prevTime  = 0;		
        hasStarted = false;	// make sure we run update() at least once before finishing
        targetAcquired = false;
    }


    @Override
    public void update() 
    {
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		imageTimestamp    	  = visionStatus.getImageTimestamp();			// TODO: modify vision code to return targets to edge of FOV
		normalizedTargetX 	  = visionStatus.getNormalizedTargetX();
		normalizedTargetWidth = visionStatus.getNormalizedTargetWidth();

		currentPose = robotState.getLatestFieldToVehicle();		

		currentTime = Timer.getFPGATimestamp();

//DEBUG: use current time instead of image time			
imageTimestamp = currentTime;			
		imageTimestamp -= Constants.kCameraLatencySeconds;		// remove camera latency
		
		// calculate target location based on *previous* robot pose
		previousPose = robotState.getFieldToVehicle(imageTimestamp);

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		pathVisionDrive(currentTime, currentPose, previousPose, imageTimestamp, normalizedTargetX, normalizedTargetWidth);	// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
		drive.driveCurve(speed, curvature, path.getSegmentMaxSpeed());
	}
	
	
	public void pathVisionDrive(double _currentTime, Pose _currentPose, Pose _previousPose, double _imageTimestamp, double _normalizedTargetX, double _normalizedTargetWidth)
	{
		// TODO: address stopping when past final segment

    	hasStarted = true;	// make sure we run update() at least once before finishing
    	
		remainingDistance = Double.MAX_VALUE;
		double maxSpeed = 0;
		double maxAccel = 0;
		
		boolean visionEnable = path.getSegmentVisionEnable(); 
		if (visionEnable)
		{
			visionDrive(_currentTime, _currentPose, _previousPose, _imageTimestamp, _normalizedTargetX, _normalizedTargetWidth);
			remainingDistance = distanceToTargetInches;
			maxSpeed = Constants.kVisionMaxVel;
			maxAccel = Constants.kVisionMaxAccel;
		}
		
		if (!visionEnable || targetAcquired == false) 
		{
			pathDrive(_currentTime, _currentPose);
			remainingDistance = path.getRemainingLength();
			maxSpeed = path.getSegmentMaxSpeed();
			maxAccel = path.getSegmentMaxAccel();
		}
		
		speedControl(_currentTime, remainingDistance, maxSpeed, maxAccel);
	}
	
	public double getSpeed() { return speed; }
	public double getCurvature() { return curvature; }
	public Path   getPath() { return path; }	// warning: not returning a defensive copy
	public double getDistanceFromPath() { return distanceFromPath; }
	public boolean getTargetAcquired() { return targetAcquired; }
	
	private void pathDrive(double _currentTime, Pose _currentPose)
	{
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
		headingToTargetRadians = robotToTarget.angle() - _currentPose.getHeadingRad();
		if (reversed)
			headingToTargetRadians -= Math.PI;
		
		curvature = 2 * Math.sin(headingToTargetRadians) / lookaheadDist;
	}

	
	
	// visionDrive() is written outside of update() to facilitate unit level testing
	public void visionDrive(double _currentTime, Pose _currentPose, Pose _previousPose, double _imageTimestamp, double _normalizedTargetX, double _normalizedTargetWidth)
	{
		distanceToTargetInches = Double.MAX_VALUE;
		
		// If we get a valid message from the Vision co-processor, update our estimate of the target location
		if (_normalizedTargetWidth > 0) 
		{
			//-----------------------------------------------------
			// Estimate target location based on previous location,
			// to compensate for latency in processing image
			//-----------------------------------------------------
			prevDistanceToTargetInches = Constants.kTargetWidthInches / (2.0*_normalizedTargetWidth*Constants.kTangentCameraHalfFOV);
			prevHeadingToTargetRadians = _previousPose.getHeadingRad() + (-_normalizedTargetX*Constants.kCameraHalfFOVRadians);
			Vector2d prevToTarget = Vector2d.magnitudeAngle(prevDistanceToTargetInches, prevHeadingToTargetRadians);
			targetLocation = _previousPose.getPosition().add(prevToTarget); 	
			
			// filter target location with exponential averaging
			if (!targetAcquired)
				avgTargetLocation = targetLocation;
			else
				avgTargetLocation = avgTargetLocation.expAverage(targetLocation, Constants.kTargetLocationFilterConstant);

			targetAcquired = true;
		}
		
		// if avgCnt > 0, then we have at least one estimate of the target location.
		// drive towards it, even if we didn't get a valid Vision co-processor message this time
		if (targetAcquired)
		{
			Vector2d robotToTarget = avgTargetLocation.sub(_currentPose.getPosition());
			distanceToTargetInches = robotToTarget.length();
			headingToTargetRadians = robotToTarget.angle() - _currentPose.getHeadingRad();
			
			//---------------------------------------------------
			// Calculate motor settings to turn towards target   
			//---------------------------------------------------
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);	// length of chord <= kVisionLookaheadDist
			curvature     = 2 * Math.sin(headingToTargetRadians) / lookaheadDist;				// curvature = 1/radius of circle (positive: turn left, negative: turn right)
		}
		else
		{
			// target not acquired -- speed/curvature will be controlled by path follower
		}
	}
	
	public void speedControl(double _currentTime, double _remainingDistance, double _maxSpeed, double _maxAccel)
	{
		//---------------------------------------------------
		// Apply speed control
		//---------------------------------------------------
		speed = _maxSpeed;
		if (reversed)
			speed = -speed;
		
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
	
	
	
    @Override
    public boolean isFinished() 
    {
    	boolean done = false;
    	
    	if (targetAcquired)
    		done = hasStarted && (remainingDistance <= Constants.kPegTargetDistanceThresholdInches);
    	else
	        done = hasStarted && (remainingDistance <= pathCompletionTolerance);
    	
    	return done;
    }

    @Override
    public void done() 
    {
		System.out.println("Finished PathFollowerWithVisionAction");
		// cleanup code, if any
        drive.stop();
    }

 
    
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("PathVision/currentTime", currentTime);
			put("PathVision/currentPoseX", currentPose.getX());
			put("PathVision/currentPoseY", currentPose.getY());
			put("PathVision/currentPoseHeadingRad", currentPose.getHeadingRad());
			put("PathVision/distanceFromPath", distanceFromPath );
			put("PathVision/lookaheadDist", lookaheadDist );
			put("PathVision/lookaheadPointX",  lookaheadPoint.getX() );
			put("PathVision/lookaheadPointY",  lookaheadPoint.getY());
			put("PathVision/headingToTargetRadians", headingToTargetRadians);

			put("PathVision/imageTime", imageTimestamp);
			put("PathVision/normalizedTargetX", normalizedTargetX);
			put("PathVision/normalizedTargetWidth", normalizedTargetWidth);
			put("PathVision/previousPoseX", previousPose.getX());
			put("PathVision/previousPoseY", previousPose.getY());
			put("PathVision/previousPoseHeadingRad", previousPose.getHeadingRad());
			put("PathVision/prevDistanceToTargetInches", prevDistanceToTargetInches);
			put("PathVision/prevHeadingToTargetRadians", prevHeadingToTargetRadians);
			put("PathVision/targetLocationX", targetLocation.getX());
			put("PathVision/targetLocationY", targetLocation.getY());
			put("PathVision/avgTargetLocationX", avgTargetLocation.getX());
			put("PathVision/avgTargetLocationY", avgTargetLocation.getY());
			put("PathVision/avgCnt", avgCnt);
			put("PathVision/distanceToTargetInches", distanceToTargetInches);

			put("PathVision/remainingDistance",  remainingDistance );
			
			put("PathVision/speed", 			 speed);
			put("PathVision/curvature", 		 curvature );
        }
    };
	
    public DataLogger getLogger() { return logger; }
}
