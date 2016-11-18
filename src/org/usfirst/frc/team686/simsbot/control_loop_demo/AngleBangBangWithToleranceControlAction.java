package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Rotation2d;

import org.usfirst.frc.team686.lib.util.*;



public class AngleBangBangWithToleranceControlAction implements Action {

    private double mTargetHeading;
    private double mVelocity;
    private Drive mDrive = Drive.getInstance();
    
    
    
    public AngleBangBangWithToleranceControlAction() 
    {
    	mTargetHeading = mDrive.getHeading() + 90.0;	// degrees
        mVelocity = 0.3;								// %
    }

    @Override
    public void start() 
    {
    	mDrive.setOpenLoop(DriveSignal.NEUTRAL);    	
    }

    @Override
    public void update() 
    {
    	// Bang-Bang Control
    	// =================
    	// Motor speed is set to full power, in the direction of the target
    	
    	double currentHeading = mDrive.getHeading();
    	
    	if (currentHeading < mTargetHeading)
    		mDrive.setOpenLoop(new DriveSignal(-mVelocity, +mVelocity));
    	else
    		mDrive.setOpenLoop(new DriveSignal(+mVelocity, -mVelocity));
    }

    @Override
    public boolean isFinished() 
    {
        boolean finished = false;
        
        if (Math.abs(mDrive.getHeading() - mTargetHeading)<1.0)
        	finished = true;
        
        return finished;
    }

    @Override
    public void done() 
    {
    	mDrive.setOpenLoop(DriveSignal.NEUTRAL);    
    }

    private double getCurrentDistance() 
    {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
