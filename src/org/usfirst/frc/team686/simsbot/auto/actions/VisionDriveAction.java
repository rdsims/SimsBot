package org.usfirst.frc.team686.simsbot.auto.actions;

import org.mini2Dx.gdx.math.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team686.lib.util.Pose;
import org.usfirst.frc.team686.lib.util.RigidTransform2d;
import org.usfirst.frc.team686.lib.util.Util;
import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.DataLogger;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;


public class VisionDriveAction implements Action 
{
	// properties needed for class
	private NetworkTable table;
	private RobotState robotState = RobotState.getInstance();
	private Drive drive = Drive.getInstance();
	private double maxSpeed;
	private double maxAccel;
	private double prevTime;
	private double prevSpeed;
	private Vector2 filteredTargetLocation;
	private int  filterCnt;

	// for logging only
	private double currentTime;
	private double imageTimestamp;
	private double normalizedTargetX;
	private double normalizedTargetWidth;
	private Pose previousPose;
	private Vector2 targetLocation;
	private Pose currentPose;
	private double distanceToTargetInches;
	private double headingToTargetRadians;
	private double lookaheadDist;
	private double curvature;
	private double speed;
	
	
	public VisionDriveAction(double _maxSpeed, double _maxAccel) 
	{
		maxSpeed = _maxSpeed;
		maxAccel = _maxAccel;
		table = NetworkTable.getTable("SmartDashboard");
	}

	@Override
	public void start() 
	{
		// setup code, if any
		filterCnt = 0;
		prevTime = 0;
		prevSpeed = 0;	// TODO: find way to not start from speed=0
	}

// TODO: combine followPath and visionDrive -- smoothly switch over to vision when 1) enabled for that segment and 2) a target was found
// Goal is not to stop when switching from path following to vision
	
	@Override
	public void update() 
	{
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		imageTimestamp    	 = table.getNumber("imageTimestamp",    0);
		normalizedTargetX 	 = table.getNumber("targetCenterX",  -999);
		normalizedTargetWidth = table.getNumber("targetWidth",    -999);

		currentTime = Timer.getFPGATimestamp();
		speed = prevSpeed;

		// If we get a valid message from the Vision co-processor, update our estimate of the target location
		if (imageTimestamp > 0) 
		{
			//-----------------------------------------------------
			// Estimate target location based on previous location,
			// to compensate for latency in processing image
			//-----------------------------------------------------
			imageTimestamp -= Constants.kCameraLatencySeconds;		// remove camera latency
			
			// calculate target location based on *previous* robot pose
			RigidTransform2d pPose = robotState.getFieldToVehicle(imageTimestamp);	// using CheesyPoof's RigidTransform2d for now  TODO: replace
			Pose previousPose = new Pose(pPose.getTranslation().getX(), pPose.getTranslation().getY(), pPose.getRotation().getRadians());
			
// DEBUG: use current pose until we... 			
// TODO: get timestamp synchronization working
pPose = robotState.getLatestFieldToVehicle();	// using CheesyPoof's RigidTransform2d for now			
previousPose = new Pose(pPose.getTranslation().getX(), pPose.getTranslation().getY(), pPose.getRotation().getRadians());

			distanceToTargetInches = Constants.kTargetWidthInches / (2.0*normalizedTargetWidth*Constants.kTangentCameraHalfFOV);
			headingToTargetRadians = previousPose.getHeadingRad() + (normalizedTargetX*Constants.kCameraHalfFOVRadians);
			Vector2 toTarget = Util.fromMagnitudeAngleRad(distanceToTargetInches, headingToTargetRadians);
			targetLocation = previousPose.getPosition().add(toTarget); 
			
			// filter target location with exponential averaging
			if (filterCnt == 0)
				filteredTargetLocation = targetLocation;
			else
				filteredTargetLocation = filterPosition(filteredTargetLocation, targetLocation, Constants.kTargetLocationFilterConstant);
			filterCnt++;
		}
		
		// if filterCnt > 0, then we have at least one estimate of the target location.
		// drive towards it, even if we didn't get a valid Vision co-processor message this time
		if (filterCnt > 0)
		{
			//---------------------------------------------------
			// Calculate path from *current* robot pose to target
			//---------------------------------------------------
			RigidTransform2d  cPose = robotState.getLatestFieldToVehicle();		// using CheesyPoof's RigidTransform2d for now.  TODO: replace		
			Pose currentPose = new Pose(cPose.getTranslation().getX(), cPose.getTranslation().getY(), cPose.getRotation().getRadians());

			//---------------------------------------------------
			// Calculate motor settings to turn towards target   
			//---------------------------------------------------
			Vector2 robotToTarget = filteredTargetLocation.sub(currentPose.getPosition());
			distanceToTargetInches = robotToTarget.len();									// distance to target
			headingToTargetRadians = robotToTarget.angleRad() - currentPose.getHeadingRad();								// change in heading from current pose to target (tangent to circle to be travelled)
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);				// length of chord <= kVisionLookaheadDist
			curvature = 2.0 * Math.sin(headingToTargetRadians) / lookaheadDist;										// curvature = 1/radius of circle (negative: turn left, positive: turn right)
		
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
			double maxBrakingSpeed = Math.sqrt(2.0 * maxAccel * distanceToTargetInches);
			if (Math.abs(speed) > maxSpeed)
				speed = Math.signum(speed) * maxBrakingSpeed;

			// apply minimum velocity limit (Talons can't track low speeds well)
			final double kMinSpeed = 4.0;
			if (Math.abs(speed) < kMinSpeed) 
				speed = Math.signum(speed) * kMinSpeed;
			
			//---------------------------------------------------
			// Send drive control
			//---------------------------------------------------
			drive.driveCurve(speed, curvature, maxSpeed);
		} 
		else 
		{
			// debug
			System.out.println("Vision NetworkTables not found");

			// invalid value: not sure if we should stop or coast
			drive.stop();
		}

		// store for next time through loop
		prevTime = currentTime;
		prevSpeed = speed;
	}

	
    // perform exponential filtering on position
    // alpha is the filtering coefficient, 0<alpha<<1
    // result will converge 63% in 1/alpha timesteps
    //                      86% in 2/alpha timesteps
    //                      95% in 3/alpha timesteps
    public static Vector2 filterPosition(Vector2 a, Vector2 b, float alpha)
    {
    	float x = (1-alpha)*a.x + alpha*b.x;
    	float y = (1-alpha)*a.y + alpha*b.y;
    	return new Vector2(x,y);
    }
	
	@Override
	public boolean isFinished() 
	{
		boolean done = (distanceToTargetInches > Constants.kPegTargetDistanceThresholdInches);
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
	
	public void log() 
	{
		DataLogger dataLogger = DataLogger.getVisionInstance();

		dataLogger.putNumber("currentTime", currentTime);
		dataLogger.putNumber("imageTime", imageTimestamp);
		dataLogger.putNumber("normalizedTargetX", normalizedTargetX);
		dataLogger.putNumber("normalizedTargetWidth", normalizedTargetWidth);
		dataLogger.putNumber("previousPoseX", previousPose.getX());
		dataLogger.putNumber("previousPoseY", previousPose.getY());
		dataLogger.putNumber("previousPoseHeadingRad", previousPose.getHeadingRad());
		dataLogger.putNumber("targetLocationX", targetLocation.x);
		dataLogger.putNumber("targetLocationY", targetLocation.y);
		dataLogger.putNumber("filteredTargetLocationX", filteredTargetLocation.x);
		dataLogger.putNumber("filteredTargetLocationY", filteredTargetLocation.y);
		dataLogger.putNumber("filterCnt", filterCnt);
		dataLogger.putNumber("currentPoseX", currentPose.getX());
		dataLogger.putNumber("currentPoseY", currentPose.getY());
		dataLogger.putNumber("currentPoseHeadingRad", currentPose.getHeadingRad());
		dataLogger.putNumber("distanceToTargetInches", distanceToTargetInches);
		dataLogger.putNumber("headingToTargetRadians", headingToTargetRadians);
		dataLogger.putNumber("lookaheadDist", lookaheadDist);
		dataLogger.putNumber("curvature", curvature);
		dataLogger.putNumber("speed", speed);
	}


}
