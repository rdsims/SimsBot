package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Rotation2d;




public class AnglePIDControlAction implements Action {

    private double mTargetHeading;
    private double mTargetDistance = 0.0;
    private double mVelocity;
    private double previous_error;
    private double integrated_error;
    private int finishCnt;
    
    private double Kp, Kd, Ki;
    
    private Drive mDrive = Drive.getInstance();
    
    
    
    public AnglePIDControlAction(double _kp, double _kd, double _ki) 
    {
    	mTargetHeading = mDrive.getHeading() + 90.0;	// degrees
        mVelocity = 0.3;								// %
        Kp = _kp;
        Kd = _kd;
        Ki = _ki;
    }

    @Override
    public void start() 
    {
    	mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    	finishCnt = 0;
    	previous_error = 0;
    	integrated_error = 0;
    }

    @Override
    public void update() 
    {
    	// PID Control
    	// =================
				
       	double currentHeading = mDrive.getHeading();
    	double error = mTargetHeading - currentHeading;
    	double change_in_error = error - previous_error;
    	integrated_error = integrated_error + error;
    	
    	double motorSpeed = (Kp * error) + (Kd * change_in_error) + (Ki * integrated_error);
     	
   		mDrive.setOpenLoop(new DriveSignal(-motorSpeed, +motorSpeed));
    
		previous_error = error;
		
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
