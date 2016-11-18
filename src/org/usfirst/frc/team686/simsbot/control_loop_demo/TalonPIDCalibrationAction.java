package org.usfirst.frc.team686.simsbot.control_loop_demo;

import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.auto.actions.*;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.lib.util.Rotation2d;




public class TalonPIDCalibrationAction implements Action {

    private double mVelocity;
    private double mHeading;
    private Drive mDrive = Drive.getInstance();
        
    
    public TalonPIDCalibrationAction() 
    {
        mVelocity = Constants.kDriveWheelCircumInches;								// inches/sec
        mHeading = 0.0;									// degrees
    }

    @Override
    public void start() 
    {
    }

    @Override
    public void update() 
    {
    	mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
    }

    @Override
    public boolean isFinished() 
    {
        boolean finished = false;
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
