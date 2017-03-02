package org.team686.simsbot.auto.actions;

import org.team686.lib.util.DataLogger;
import org.team686.simsbot.command_status.DriveStatus;
import org.team686.simsbot.subsystems.Drive;

// TODO: add point turn action using gyro and motion profiling
public class PointTurnAction implements Action
{
	
    private double targetHeadingDeg;
    private final double headingCompletionToleranceDeg = 1.0;
    private Drive drive = Drive.getInstance();
    private DriveStatus driveStatus = DriveStatus.getInstance();

    public PointTurnAction(double _targetHeadingDeg)
    {
        targetHeadingDeg = _targetHeadingDeg;
    }

    @Override
    public void start() 
    {
        drive.setVelocityHeadingSetpoint(0, targetHeadingDeg);
    }

    @Override
    public void update() 
    {
    	// Nothing to do.  LoopController will call 
    	// drive.velocityControlLoop.onLoop()
    	// which will tend to the PID
    }

    @Override
    public boolean isFinished() 
    {
    	return (Math.abs(driveStatus.getHeadingDeg()) < headingCompletionToleranceDeg);
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
			put("DriveCmd/talonMode", driveStatus.getTalonControlMode().toString() );
			put("DriveCmd/left", drive.getCommand().getLeftMotor() );
			put("DriveCmd/right", drive.getCommand().getRightMotor() );
    		put("DriveStatus/TalonControlMode", driveStatus.getTalonControlMode().toString() );
			put("DriveStatus/lSpeed", driveStatus.getLeftSpeedInchesPerSec() );
			put("DriveStatus/rSpeed", driveStatus.getRightSpeedInchesPerSec() );
    		put("DriveStatus/lDistance", driveStatus.getLeftDistanceInches() );
    		put("DriveStatus/rDistance", driveStatus.getRightDistanceInches() );
    		put("DriveStatus/Heading", driveStatus.getHeadingDeg() );
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    	
}
