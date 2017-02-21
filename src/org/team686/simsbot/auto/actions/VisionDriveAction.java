package org.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.DataLogger;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.command_status.VisionStatus;
import org.team686.simsbot.subsystems.Drive;


public class VisionDriveAction implements Action 
{
	// properties needed for class
	public VisionStatus visionStatus = VisionStatus.getInstance();
	public RobotState robotState = RobotState.getInstance();
	public Drive drive = Drive.getInstance();
	public double maxSpeed;
	public double maxAccel;
	public double prevTime;
	public double prevSpeed;
	public Vector2d avgTargetLocation = new Vector2d(0,0);
	public int  avgCnt;

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
	public double headingToTargetRadians;
	public double lookaheadDist;
	public double curvature;
	public double speed;
	
	
	public VisionDriveAction(double _maxSpeed, double _maxAccel) 
	{
		System.out.println("Starting VisionDriveAction");
		maxSpeed = _maxSpeed;
		maxAccel = _maxAccel;
	}

	@Override
	public void start() 
	{
		// setup code, if any
		System.out.println("Starting VisionDriveAction");
		avgCnt = 0;
		prevTime = 0;
		prevSpeed = 0;	// TODO: find way to not start from speed=0
		distanceToTargetInches = Double.MAX_VALUE;	// make sure we don't trigger isFinished() without running update() at least once
	}

// TODO: combine followPath and visionDrive -- smoothly switch over to vision when 1) enabled for that segment and 2) a target was found
// Goal is not to stop when switching from path following to vision
	
	@Override
	public void update() 
	{
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		imageTimestamp    	  = visionStatus.getImageTimestamp();
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
		visionDrive(currentTime, currentPose, previousPose, imageTimestamp, normalizedTargetX, normalizedTargetWidth);

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
		drive.driveCurve(speed, curvature, maxSpeed);
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
			if (avgCnt == 0)
				avgTargetLocation = targetLocation;
			else
				avgTargetLocation = avgTargetLocation.expAverage(targetLocation, Constants.kTargetLocationFilterConstant);
			avgCnt++;
		}
		
		// if avgCnt > 0, then we have at least one estimate of the target location.
		// drive towards it, even if we didn't get a valid Vision co-processor message this time
		if (avgCnt > 0)
		{
			Vector2d robotToTarget = avgTargetLocation.sub(_currentPose.getPosition());
			distanceToTargetInches = robotToTarget.length();
			headingToTargetRadians = robotToTarget.angle() - _currentPose.getHeadingRad();
			
			//---------------------------------------------------
			// Apply speed control
			//---------------------------------------------------
			speed = maxSpeed;	// goal is to get to maximum speed
			double dt = _currentTime - prevTime; 
			
			// apply acceleration limits
			double accel = (speed - prevSpeed) / dt;
			if (accel > maxAccel)
				speed = prevSpeed + maxAccel * dt;
			else if (accel < -maxAccel)
				speed = prevSpeed - maxAccel * dt;

			// apply braking distance limits
			// vf^2 = v^2 + 2*a*d   Solve for v, given vf=0, configured a, and measured d
			double stoppingDistance = Math.max(distanceToTargetInches - Constants.kPegTargetDistanceThresholdInches, 0);
			double maxBrakingSpeed = Math.sqrt(2.0 * maxAccel * stoppingDistance);
			if (Math.abs(speed) > maxBrakingSpeed)
				speed = Math.signum(speed) * maxBrakingSpeed;

			// apply minimum velocity limit (Talons can't track low speeds well)
			final double kMinSpeed = 4.0;
			if (Math.abs(speed) < kMinSpeed) 
				speed = Math.signum(speed) * kMinSpeed;


			//---------------------------------------------------
			// Calculate motor settings to turn towards target   
			//---------------------------------------------------
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);	// length of chord <= kVisionLookaheadDist
			curvature     = 2 * Math.sin(headingToTargetRadians) / lookaheadDist;				// curvature = 1/radius of circle (positive: turn left, negative: turn right)

		}
		else
		{
			speed = prevSpeed;
			curvature = 0;
		}
			
		// store for next time through loop
		prevTime = _currentTime;
		prevSpeed = speed;			// TODO: use measured speed instead of computed speed
	}

	public double getSpeed() { return speed; }
	public double getCurvature() { return curvature; }
	
	
	@Override
	public boolean isFinished() 
	{
		boolean done = (distanceToTargetInches <= Constants.kPegTargetDistanceThresholdInches);
		if (done) 
		{
			System.out.println("Finished VisionDriveAction");
		}
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
			put("VisionDrive/currentPoseX", currentPose.getX());
			put("VisionDrive/currentPoseY", currentPose.getY());
			put("VisionDrive/currentPoseHeadingRad", currentPose.getHeadingRad());
			put("VisionDrive/distanceToTargetInches", distanceToTargetInches);
			put("VisionDrive/headingToTargetRadians", headingToTargetRadians);
			put("VisionDrive/lookaheadDist", lookaheadDist);
			put("VisionDrive/curvature", curvature);
			put("VisionDrive/speed", speed);
	    }
    };
	
    public DataLogger getLogger() { return logger; }


}
