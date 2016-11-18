package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.util.Rotation2d;




public class DistancePControlAction implements Action {

	private double mTargetHeading = 0;
    private double mTargetDistance;
    private double mVelocity;
    private double mHeading;
    private Drive mDrive = Drive.getInstance();
    
    double Kp, Kd, Ki;
    int finishCnt;
    
    enum DemoState { DEMO, WAIT, RETURN_HOME };
    DemoState mDemoState;
    
    
    public DistancePControlAction(double _kp) 
    {
    	mTargetDistance = getCurrentDistance() + 36.0;	// inches
        mVelocity = 24.0;								// inches/sec
        mHeading = 0.0;									// degrees
        mDemoState = DemoState.DEMO;
        Kp = _kp;
        
    }

    @Override
    public void start() 
    {
    	finishCnt = 0;
    }

    @Override
    public void update() 
    {

    	switch (mDemoState)
    	{
    	case DEMO:
        	// Proportional Control
        	// =================
        	// Motor speed is proportional to distance from target
    		
        	double currentDistance = getCurrentDistance();
        	double error = mTargetDistance - currentDistance;
        	double motorSpeed = Kp * error;
        	
    		mDrive.setVelocityHeadingSetpoint(motorSpeed, Rotation2d.fromDegrees(mHeading));

            if (Math.abs(currentDistance - mTargetDistance)<0.25)
            	finishCnt++;
            else
            	finishCnt = 0;
            
            if (finishCnt>100)
            	mDemoState = DemoState.WAIT;
        	break;
        	
        	
    	case WAIT:
            mDrive.setVelocitySetpoint(0, 0);
            try {
				Thread.sleep(10*1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
   			mDemoState = DemoState.RETURN_HOME;
    		break;
    	
    		
    		
    	case RETURN_HOME:
       		mDrive.setVelocityHeadingSetpoint(-12.0, Rotation2d.fromDegrees(mHeading));
    		
    	}
    	
    	SmartDashboard.putNumber("Ctrl Demo Distance Error", mTargetDistance - getCurrentDistance());
    	SmartDashboard.putNumber("Ctrl Demo Heading Error",  mTargetHeading - mDrive.getHeading());
    }

    @Override
    public boolean isFinished() 
    {
        boolean finished = false;
        
        if ((mDemoState == DemoState.RETURN_HOME) && (getCurrentDistance() <= 0.0))
        	finished = true;
        
        return finished;
    }

    @Override
    public void done() 
    {
        mDrive.setVelocitySetpoint(0, 0);
    }

    private double getCurrentDistance() 
    {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
