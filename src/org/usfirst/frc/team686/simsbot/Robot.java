
package org.usfirst.frc.team686.simsbot;

import java.io.File;
import java.io.IOException;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.util.*;

import org.usfirst.frc.team686.simsbot.JoystickControls;
import org.usfirst.frc.team686.simsbot.loops.LoopList;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	PowerDistributionPanel pdp = new PowerDistributionPanel();
	
	Drive drive = Drive.getInstance();
	JoystickControls controls = JoystickControls.getInstance();
	
	DataLogger dataLogger = DataLogger.getInstance();
    
	LoopList loopList  = new LoopList();
	
	
   public Robot()
   {
	   CrashTracker.logRobotConstruction();
   }
	
	
   public void zeroAllSensors() {
        drive.zeroSensors();
    }

	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	@Override
    public void robotInit() 
    {
		try
		{
            CrashTracker.logRobotInit();	
	
	       // Reset all state
	        zeroAllSensors();
	        
	        // Configure LoopLists
	        loopList.register(drive.getLoop());
	
	    	// Set dataLogger and Time information
	    	TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));
	
	    	// Determine folder for log files
	    	dataLogger.findLogDirectory();
		}
		catch (Throwable t)
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
    }
    
	
    public void stopAll() 
    {
        drive.stop();
//        mSuperstructure.stop();
    }

	
	public void log()
	{
		if (dataLogger.shouldLogData())
		{
			dataLogger.addDataItem("lMotorCurrent",  pdp.getCurrent(15));
			dataLogger.addDataItem("rMotorCurrent",  pdp.getCurrent(0));
			drive.log();
			dataLogger.saveDataItems();
		}
	}	
	
	
	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/
	
	
	 @Override
	 public void disabledInit() 
	 {
        try 
        {
            CrashTracker.logDisabledInit();
 /*
            if (mAutoModeExecuter != null) 
            {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;
*/
            
            loopList.stop();

            drive.setOpenLoop(DriveSignal.NEUTRAL);
            drive.setBrakeMode(true);
            // Stop all actuators
            stopAll();
            
        } 
        catch (Throwable t) 
        {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }	
        
    
    @Override
    public void disabledPeriodic() 
    {
    	try
    	{
    		stopAll();
    		drive.resetEncoders();
    	}
    	catch (Throwable t)
    	{
            CrashTracker.logThrowableCrash(t);
            throw t;
    	}
    }
	
    
    

	/****************************************************************
	 * AUTONOMOUS MODE
	 ****************************************************************/
       
	@Override
    public void autonomousInit() 
    {
		try
		{
			CrashTracker.logAutoInit();
			
	        // Reset all sensors
	        zeroAllSensors();

            loopList.start();
		}
    	catch (Throwable t)
    	{
            CrashTracker.logThrowableCrash(t);
            throw t;
    	}
    }

     
     @Override
    public void autonomousPeriodic() 
    {
    	 try 
    	 {
	        log();        
//             outputAllToSmartDashboard();
//             updateDriverFeedback();
         } 
    	 catch (Throwable t) 
    	 {
             CrashTracker.logThrowableCrash(t);
             throw t;
         }
     }
       
    
    
    
    
	/****************************************************************
	 * TELEOP MODE
	 ****************************************************************/
    
    @Override
    public void teleopInit()
    {
    	try
    	{
            CrashTracker.logTeleopInit();

            // Reset drive
            drive.resetEncoders();

           // Configure loopList
            loopList.start();
            
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            drive.setBrakeMode(false);
        }
    	catch (Throwable t) 
    	{
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    		
    }
    
    
    @Override
    public void teleopPeriodic() 
    {
    	try
    	{
            drive.setOpenLoop(drive.tankDrive( controls.getThrottle(), controls.getTurn() ));
            
            log();           		
    	}
    	catch (Throwable t) 
    	{
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
   	
    } 
}