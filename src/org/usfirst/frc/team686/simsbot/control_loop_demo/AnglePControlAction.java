package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Rotation2d;




public class AnglePControlAction implements Action {

    private double mTargetHeading;
    private double mTargetDistance = 0.0;
    private double mVelocity;
    private Drive mDrive = Drive.getInstance();
    
    private double Kp, Kd, Ki;
    
    private int finishCnt;
    
    
    public AnglePControlAction(double _kp) 
    {
    	mTargetHeading = mDrive.getHeading() + 90.0;	// degrees
        mVelocity = 0.3;								// %
        Kp = _kp;
    }

    @Override
    public void start() 
    {
    	finishCnt = 0;
    	mDrive.setOpenLoop(DriveSignal.NEUTRAL);    	
    }

    @Override
    public void update() 
    {
    	// Proportional Control
    	// =================
    	// Motor speed is proportional to distance from target
		
       	double currentHeading = mDrive.getHeading();
    	double error = mTargetHeading - currentHeading;
    	double motorSpeed = Kp * error;
    	
     	
   		mDrive.setOpenLoop(new DriveSignal(-motorSpeed, +motorSpeed));
   		
    	SmartDashboard.putNumber("Ctrl Demo Distance Error", mTargetDistance - getCurrentDistance());
    	SmartDashboard.putNumber("Ctrl Demo Heading Error",  mTargetHeading - mDrive.getHeading());
   		
    }

    @Override
    public boolean isFinished() 
    {
        boolean finished = false;
        
        if (Math.abs(mDrive.getHeading() - mTargetHeading)<0.1)
        	finishCnt++;
        else
        	finishCnt = 0;
        
        if (finishCnt>100)
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
