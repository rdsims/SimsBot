package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.util.Rotation2d;




public class DistancePDControlAction implements Action {

	private double mTargetHeading = 0;
    private double mTargetDistance;
    private double mVelocity;
    private double mHeading;
    private double previous_error;
    private int finishCnt;
    
    private double Kp, Kd, Ki;
    
    private Drive mDrive = Drive.getInstance();
    
    enum DemoState { DEMO, WAIT, RETURN_HOME };
    DemoState mDemoState;
    
    
    public DistancePDControlAction(double _kp, double _kd) 
    {
    	mTargetDistance = getCurrentDistance() + 36.0;	// inches
        mVelocity = 24.0;								// inches/sec
        mHeading = 0.0;									// degrees
        mDemoState = DemoState.DEMO;
        Kp = _kp;
        Kd = _kd;
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
        	// Proportional + Derivative Control
        	// =================================
        	// A PID loop with no Integrator feedback (a very typical setting for PIDs)

    		// Kp can increase from setting in plain proportional controller because 
    		// derivative term will slow it down as you approach the target    		
    		
        	double currentDistance = getCurrentDistance();
        	double error = mTargetDistance - currentDistance;
        	double change_in_error = error - previous_error;
        	double motorSpeed = (Kp * error) + (Kd * change_in_error);
        	
    		mDrive.setVelocityHeadingSetpoint(motorSpeed, Rotation2d.fromDegrees(mHeading));

    		previous_error = error;
    		
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
