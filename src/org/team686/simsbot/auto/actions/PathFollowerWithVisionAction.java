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
	public double maxSpeed;
	public double maxAccel;
	public Vector2d avgTargetLocation = new Vector2d(0,0);
	public int  avgCnt;

	public double distanceFromPath;
	public double lookaheadDist;
	public Vector2d lookaheadPoint;
	public double headingToTargetRadians;
	public double remainingLength;
	
	
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
		System.out.println("Starting PathFollowerAction");
		prevSpeed = 0;
		prevTime  = 0;		
        hasStarted = false;	// make sure we run update() at least once before finishing
        targetAcquired = false;
    }


    @Override
    public void update() 
    {
    	hasStarted = true;	// make sure we run update() at least once before finishing
    	
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

		boolean visionEnable = path.getSegmentVisionEnable(); 
		if (visionEnable)
		{
			visionDrive(_currentTime, _currentPose, _previousPose, _imageTimestamp, _normalizedTargetX, _normalizedTargetWidth);
		}
		
		if (!visionEnable || targetAcquired == false) 
		{
			pathDrive(_currentTime, _currentPose);
		}

		//---------------------------------------------------
		// Apply speed control
		//---------------------------------------------------
		double maxSpeed = path.getSegmentMaxSpeed();	
		double maxAccel = path.getSegmentMaxAccel();
		
		speed = maxSpeed;
		if (reversed)
			speed = -speed;
		
		double dt = _currentTime - prevTime;
		
		// apply acceleration limits
		double accel = (speed - prevSpeed) / dt;
		if (accel > maxAccel)
			speed = prevSpeed + maxAccel * dt;
		else if (accel < -maxAccel)
			speed = prevSpeed - maxAccel * dt;

		// apply braking distance limits
		// vf^2 = v^2 + 2*a*d   Solve for v, given vf=0, configured a, and measured d
		double stoppingDistance = remainingLength;
		double maxBrakingSpeed = Math.sqrt(2.0 * maxAccel * stoppingDistance);
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
	
	public double getSpeed() { return speed; }
	public double getCurvature() { return curvature; }
	
	
	private void pathDrive(double _currentTime, Pose _currentPose)
	{
		//---------------------------------------------------
		// Find Lookahead Point
		//---------------------------------------------------
		distanceFromPath = path.update(_currentPose.getPosition());
		lookaheadPoint = path.getLookaheadPoint(_currentPose.getPosition(), distanceFromPath);
		remainingLength = path.getRemainingLength();
		
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
	
	
	
    @Override
    public boolean isFinished() 
    {
    	boolean done = false;
    	
    	if (targetAcquired)
    	{
    		done = (distanceToTargetInches <= Constants.kPegTargetDistanceThresholdInches);
    	}
    	else
    	{
			remainingLength = path.getRemainingLength();
	        done = hasStarted && (remainingLength <= pathCompletionTolerance);
    	}
    	
        if (done) 
			System.out.println("Finished PathFollowerAction");
        return done;
    }

    @Override
    public void done() 
    {
		// cleanup code, if any
        drive.stop();
    }

 
    
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("VisionDrive/currentTime", currentTime);
			put("VisionDrive/currentPoseX", currentPose.getX());
			put("VisionDrive/currentPoseY", currentPose.getY());
			put("VisionDrive/currentPoseHeadingRad", currentPose.getHeadingRad());
			put("PathFollower/distanceFromPath", distanceFromPath );
			put("PathFollower/lookaheadDist", lookaheadDist );
			put("PathFollower/lookaheadPointX",  lookaheadPoint.getX() );
			put("PathFollower/lookaheadPointY",  lookaheadPoint.getY());
			put("VisionDrive/headingToTargetRadians", headingToTargetRadians);
			put("PathFollower/remainingLength",  remainingLength );

			put("VisionDrive/imageTime", imageTimestamp);
			put("VisionDrive/normalizedTargetX", normalizedTargetX);
			put("VisionDrive/normalizedTargetWidth", normalizedTargetWidth);
			put("VisionDrive/previousPoseX", previousPose.getX());
			put("VisionDrive/previousPoseY", previousPose.getY());
			put("VisionDrive/previousPoseHeadingRad", previousPose.getHeadingRad());
			put("VisionDrive/prevDistanceToTargetInches", prevDistanceToTargetInches);
			put("VisionDrive/prevHeadingToTargetRadians", prevHeadingToTargetRadians);
			put("VisionDrive/targetLocationX", targetLocation.getX());
			put("VisionDrive/targetLocationY", targetLocation.getY());
			put("VisionDrive/avgTargetLocationX", avgTargetLocation.getX());
			put("VisionDrive/avgTargetLocationY", avgTargetLocation.getY());
			put("VisionDrive/avgCnt", avgCnt);
			put("VisionDrive/distanceToTargetInches", distanceToTargetInches);

			put("PathFollower/speed", 			 speed);
			put("PathFollower/curvature", 		 curvature );
        }
    };
	
    public DataLogger getLogger() { return logger; }
}
