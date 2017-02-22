package org.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.RobotState;
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
public class PathFollowerAction implements Action 
{
    private Drive drive;

	Path path;

    private boolean hasStarted;
	
	double nominalLookaheadDist;
	double maxAccel;
	double nominal_dt;
	double pathCompletionTolerance;

	public double currentTime;
	public Pose   currentPose;
	
	public double distanceFromPath;
	public double lookaheadDist;
	public Vector2d lookaheadPoint;
	public double headingToTargetRadians;
	public double remainingLength;
	public double speed;
	public double curvature;

	private double prevSpeed;
	private double prevTime;

	
    public PathFollowerAction(Path _path) 
    {
        drive = Drive.getInstance();

        path = _path;

		nominalLookaheadDist = Constants.kPathFollowingLookahead;
		maxAccel = Constants.kPathFollowingMaxAccel;
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
    }


    @Override
    public void update() 
    {
    	hasStarted = true;	// make sure we run update() at least once before finishing
    	
		currentTime = Timer.getFPGATimestamp();
		currentPose = RobotState.getInstance().getLatestFieldToVehicle();
		
		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		pathDrive(currentTime, currentPose);	// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
		drive.driveCurve(speed, curvature, Constants.kPathFollowingMaxVel);
	}
	
	
	private void pathDrive(double _currentTime, Pose _currentPose)
	{
		// TODO: address stopping when past final segment
		// TODO: add vision
		
		boolean reversed = path.getReversed();
		
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

		//---------------------------------------------------
		// Apply speed control
		//---------------------------------------------------
		speed = path.getSegmentSpeed();	
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

    @Override
    public boolean isFinished() 
    {
		remainingLength = path.getRemainingLength();
        boolean done = hasStarted && (remainingLength <= pathCompletionTolerance);
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
			put("PathFollower/speed", 			 speed);
			put("PathFollower/curvature", 		 curvature );
 	    }
    };
	
    public DataLogger getLogger() { return logger; }
}
