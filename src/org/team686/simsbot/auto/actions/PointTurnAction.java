package org.team686.simsbot.auto.actions;

import org.team686.lib.util.DataLogger;
import org.team686.lib.util.Vector2d;
import org.team686.lib.util.Kinematics.WheelSpeed;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.subsystems.Drive;

import edu.wpi.first.wpilibj.Timer;

public class PointTurnAction implements Action
{
	
    private double thetaTarget;
    private Drive drive = Drive.getInstance();
    private RobotState robotState = RobotState.getInstance();

    private double thetaGyro;
    private double thetaRemaining;
    private double distanceRemaining;
    
	private final double turnRadius = Constants.kTrackWidthInches / 2;
    
    private double speed;
    private WheelSpeed wheelSpeed;
    
	private double prevSpeed;
	private double prevTime;
	
    public PointTurnAction(double _targetHeadingDeg)
    {
        thetaTarget = _targetHeadingDeg * Vector2d.degreesToRadians;
    }

    @Override
    public void start() 
    {
    	prevSpeed = 0.0;
    	prevTime = Timer.getFPGATimestamp();
    }

    
    @Override
    public void update() 
    {
    	// calculate distance remaining for each wheel
    	thetaGyro = robotState.getLatestFieldToVehicle().getHeading();
    	thetaRemaining = Vector2d.normalizeAngle( thetaTarget - thetaGyro );
    	distanceRemaining = Math.abs(turnRadius * thetaRemaining);
    	
    	// calculate speed for left/right wheels, considering max velocity & acceleration
    	double currentTime = Timer.getFPGATimestamp();
    	speedControl(currentTime, distanceRemaining, Constants.kPointTurnMaxVel, Constants.kPointTurnMaxAccel);
    	speed *= Math.signum(thetaRemaining);	// positive: right turn, negative: left turn
   		wheelSpeed = new WheelSpeed(-speed, +speed);

   		// send drive control command
        drive.setVelocitySetpoint(wheelSpeed);
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

		// apply minimum velocity limit
		final double kMinSpeed = Constants.kPointTurnMinSpeed;
		if (Math.abs(speed) < kMinSpeed) 
			speed = Math.signum(speed) * kMinSpeed;

		// store for next time through loop	
		prevTime = _currentTime;
		prevSpeed = speed;
	}
    
    
    @Override
    public boolean isFinished() 
    {
    	thetaGyro = robotState.getLatestFieldToVehicle().getHeading();
    	thetaRemaining = Vector2d.normalizeAngle( thetaTarget - thetaGyro );
    	
    	return (Math.abs(thetaRemaining) < Constants.kPointTurnCompletionTolerance);
    }

    @Override
    public void done()
    {
        drive.setVelocitySetpoint(0, 0);
    }

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("AutoAction", "PointTurn" );
			put("PointTurn/thetaGyro", thetaGyro );
			put("PointTurn/thetaRemaining", thetaRemaining );
			put("PointTurn/distanceRemaining", distanceRemaining );
			put("PointTurn/speed", speed );
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    	
}

