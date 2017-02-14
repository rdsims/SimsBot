package org.team686.simsbot.auto.actions;

import org.mini2Dx.gdx.math.*;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.Pose;
import org.team686.lib.util.RigidTransform2d;
import org.team686.lib.util.Util;
import org.team686.simsbot.Constants;
import org.team686.simsbot.DataLogger;
import org.team686.simsbot.RobotState;
import org.team686.simsbot.VisionStatus;
import org.team686.simsbot.subsystems.Drive;


public class VisionDriveAction implements Action 
{
	// properties needed for class
	private VisionStatus visionStatus = VisionStatus.getInstance();
	private RobotState robotState = RobotState.getInstance();
	private Drive drive = Drive.getInstance();
	private double maxSpeed;
	private double maxAccel;
	private double prevTime;
	private double prevSpeed;
	private Vector2 avgTargetLocation = Vector2.Zero;
	private int  avgCnt;

	// for logging only
	private double currentTime;
	private double imageTimestamp;
	private double normalizedTargetX;
	private double normalizedTargetWidth;
	private Pose previousPose = Pose.DEFAULT;
	private double prevDistanceToTargetInches;
	private double prevHeadingToTargetRadians;
	private Vector2 targetLocation = Vector2.Zero;
	private Pose currentPose = Pose.DEFAULT;
	private double distanceToTargetInches;
	private double headingToTargetRadians;
	private double lookaheadDist;
	private double curvature;
	private double speed;
	
	
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

		RigidTransform2d  cPose = robotState.getLatestFieldToVehicle();		// using CheesyPoof's RigidTransform2d for now.  TODO: replace		
		Pose currentPose = new Pose(cPose.getTranslation().getX(), cPose.getTranslation().getY(), cPose.getRotation().getRadians());

		currentTime = Timer.getFPGATimestamp();

//DEBUG: use current time instead of image time			
imageTimestamp = currentTime;			
		imageTimestamp -= Constants.kCameraLatencySeconds;		// remove camera latency
		
		// calculate target location based on *previous* robot pose
		RigidTransform2d pPose = robotState.getFieldToVehicle(imageTimestamp);	// using CheesyPoof's RigidTransform2d for now  TODO: replace
		Pose previousPose = new Pose(pPose.getTranslation().getX(), pPose.getTranslation().getY(), pPose.getRotation().getRadians());

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		visionDrive(currentTime, imageTimestamp, normalizedTargetX, normalizedTargetWidth, currentPose, previousPose);

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
		drive.driveCurve(speed, curvature, maxSpeed);
	}
	
	// visionDrive() is written outside of update() to facilitate unit level testing
	public void visionDrive(double currentTime, double imageTimestamp, double normalizedTargetX, double normalizedTargetWidth, Pose currentPose, Pose previousPose)
	{
		// If we get a valid message from the Vision co-processor, update our estimate of the target location
		if (normalizedTargetWidth > 0) 
		{
			//-----------------------------------------------------
			// Estimate target location based on previous location,
			// to compensate for latency in processing image
			//-----------------------------------------------------
			prevDistanceToTargetInches = Constants.kTargetWidthInches / (2.0*normalizedTargetWidth*Constants.kTangentCameraHalfFOV);
			prevHeadingToTargetRadians = previousPose.getHeadingRad() + (-normalizedTargetX*Constants.kCameraHalfFOVRadians);
			Vector2 prevToTarget = Util.fromMagnitudeAngleRad(prevDistanceToTargetInches, prevHeadingToTargetRadians);
			targetLocation = (new Vector2(previousPose.getPosition())).add(prevToTarget); 	// make a copy of previousPose so that add doesn't change it
			
			// filter target location with exponential averaging
			if (avgCnt == 0)
				avgTargetLocation = targetLocation;
			else
				Util.expAverage(avgTargetLocation, targetLocation, Constants.kTargetLocationFilterConstant);
			avgCnt++;
		}
		
		// if avgCnt > 0, then we have at least one estimate of the target location.
		// drive towards it, even if we didn't get a valid Vision co-processor message this time
		if (avgCnt > 0)
		{
			Vector2 robotToTarget = (new Vector2(avgTargetLocation)).sub(currentPose.getPosition());	// make a copy of avgTargetLocation so that sub doesn't change it
			distanceToTargetInches = robotToTarget.len();									// distance to target
			headingToTargetRadians = robotToTarget.angleRad() - currentPose.getHeadingRad();								// change in heading from current pose to target (tangent to circle to be travelled)

			//---------------------------------------------------
			// Apply speed control
			//---------------------------------------------------
			speed = maxSpeed;	// goal is to get to maximum speed
			double dt = currentTime - prevTime; 
			
			// apply acceleration limits
			double accel = (speed - prevSpeed) / dt;
			if (accel > maxAccel)
				speed = prevSpeed + maxAccel * dt;
			else if (accel < -maxAccel)
				speed = prevSpeed - maxAccel * dt;

			// apply braking distance limits
			// vf^2 = v^2 + 2*a*d   Solve for v, given vf=0, configured a, and measured d
			double stoppingDistance = distanceToTargetInches - Constants.kPegTargetDistanceThresholdInches;
			double maxBrakingSpeed = Math.sqrt(2.0 * maxAccel * stoppingDistance);
			if (Math.abs(speed) > maxSpeed)
				speed = Math.signum(speed) * maxBrakingSpeed;

			// apply minimum velocity limit (Talons can't track low speeds well)
			final double kMinSpeed = 4.0;
			if (Math.abs(speed) < kMinSpeed) 
				speed = Math.signum(speed) * kMinSpeed;


			//---------------------------------------------------
			// Calculate motor settings to turn towards target   
			//---------------------------------------------------
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);				// length of chord <= kVisionLookaheadDist
			curvature = 2.0 * Math.sin(headingToTargetRadians) / lookaheadDist;										// curvature = 1/radius of circle (negative: turn left, positive: turn right)

		}
		else
		{
			speed = prevSpeed;
			curvature = 0;
		}
			
		// store for next time through loop
		prevTime = currentTime;
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
			putNumber("VisionDrive/currentTime", currentTime);
			putNumber("VisionDrive/imageTime", imageTimestamp);
			putNumber("VisionDrive/normalizedTargetX", normalizedTargetX);
			putNumber("VisionDrive/normalizedTargetWidth", normalizedTargetWidth);
			putNumber("VisionDrive/previousPoseX", previousPose.getX());
			putNumber("VisionDrive/previousPoseY", previousPose.getY());
			putNumber("VisionDrive/previousPoseHeadingRad", previousPose.getHeadingRad());
			putNumber("VisionDrive/prevDistanceToTargetInches", prevDistanceToTargetInches);
			putNumber("VisionDrive/prevHeadingToTargetRadians", prevHeadingToTargetRadians);
			putNumber("VisionDrive/targetLocationX", targetLocation.x);
			putNumber("VisionDrive/targetLocationY", targetLocation.y);
			putNumber("VisionDrive/avgTargetLocationX", avgTargetLocation.x);
			putNumber("VisionDrive/avgTargetLocationY", avgTargetLocation.y);
			putNumber("VisionDrive/avgCnt", avgCnt);
			putNumber("VisionDrive/currentPoseX", currentPose.getX());
			putNumber("VisionDrive/currentPoseY", currentPose.getY());
			putNumber("VisionDrive/currentPoseHeadingRad", currentPose.getHeadingRad());
			putNumber("VisionDrive/distanceToTargetInches", distanceToTargetInches);
			putNumber("VisionDrive/headingToTargetRadians", headingToTargetRadians);
			putNumber("VisionDrive/lookaheadDist", lookaheadDist);
			putNumber("VisionDrive/curvature", curvature);
			putNumber("VisionDrive/speed", speed);
        }
    };
    
    public DataLogger getLogger() { return logger; }
	

}
