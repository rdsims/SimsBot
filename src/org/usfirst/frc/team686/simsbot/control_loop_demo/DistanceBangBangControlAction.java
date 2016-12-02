package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.lib.util.Rotation2d;




public class DistanceBangBangControlAction implements Action {

    private double mTargetDistance;
    private double mVelocity;
    private double mHeading;
    private Drive mDrive = Drive.getInstance();
    
    enum DemoState { DEMO, WAIT, RETURN_HOME };
    DemoState mDemoState;
    
    
    public DistanceBangBangControlAction() 
    {
    	mTargetDistance = getCurrentDistance() + 48.0;	// inches
        mVelocity = 24.0;								// inches/sec
        mHeading = 0.0;									// degrees
        mDemoState = DemoState.DEMO;
    }

    @Override
    public void start() 
    {
    }

    @Override
    public void update() 
    {
    	switch (mDemoState)
    	{
    	case DEMO:
        	double error = mTargetDistance - getCurrentDistance();
        	// Bang-Bang Control
        	// =================
        	// Motor speed is set to full power, in the direction of the target
        	
        	double currentDistance = getCurrentDistance();
        	
        	if (error > .25)
        	{
        		mDrive.setVelocityHeadingSetpoint(+mVelocity, Rotation2d.fromDegrees(mHeading));
        	}
        	else if (error < -.25)
        	{
        		mDrive.setVelocityHeadingSetpoint(-mVelocity, Rotation2d.fromDegrees(mHeading));
        	}
        	else
        		mDemoState = DemoState.WAIT;
        		
            if (currentDistance == mTargetDistance)
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
