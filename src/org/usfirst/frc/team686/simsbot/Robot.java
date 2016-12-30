
package org.usfirst.frc.team686.simsbot;

import java.io.File;
import java.io.IOException;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.joystick.*;
import org.usfirst.frc.team686.lib.util.*;

import org.usfirst.frc.team686.simsbot.auto.AutoModeExecuter;
import org.usfirst.frc.team686.simsbot.loops.LoopList;
import org.usfirst.frc.team686.simsbot.loops.RobotStateEstimator;
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
	AutoModeExecuter mAutoModeExecuter = null;
	
	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
	DataLogger dataLogger = DataLogger.getInstance();
    RobotState mRobotState = RobotState.getInstance();
    
	LoopList loopList  = new LoopList();
	
    SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();

    enum OperationalMode 
    { 
    	DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);
    
    	private int val;
    	private OperationalMode(int val) { this.val = val; }
    	public int getVal() { return val; }
    }

    
    
    
   public Robot()
   {
	   CrashTracker.logRobotConstruction();
   }
	
	
   public void zeroAllSensors() {
        drive.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d(), new Rotation2d());
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
	        loopList.register(RobotStateEstimator.getInstance());
	        
	    	// Set dataLogger and Time information
	    	TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));
	
	        mSmartDashboardInteractions.initWithDefaults();

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
		drive.log();
		mRobotState.log();
		loopList.log();
		dataLogger.saveDataItems();
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
            if (mAutoModeExecuter != null) 
            {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;
            
            loopList.stop();

            drive.setOpenLoop(DriveSignal.NEUTRAL);
            drive.setBrakeMode(true);
            
            stopAll();	// Stop all actuators

      		dataLogger.putNumber("OperationalMode", OperationalMode.DISABLED.getVal());
    		DataLogger.setOutputMode(DataLogger.OutputMode.SMARTDASHBOARD_ONLY);
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
	        log();            		
    		
    		System.gc(); 	// runs garbage collector
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
            if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;
			
	        // Reset all sensors
    		drive.resetEncoders();
	        zeroAllSensors();

      		dataLogger.putNumber("OperationalMode", OperationalMode.AUTONOMOUS.getVal());
    		DataLogger.setOutputMode(DataLogger.OutputMode.SMARTDASHBOARD_AND_FILE);
	        
            loopList.start();

            mAutoModeExecuter = new AutoModeExecuter();
            mAutoModeExecuter.setAutoMode(mSmartDashboardInteractions.getSelectedAutonMode());
            mAutoModeExecuter.start();
                        
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

            // Select joystick control method
            controls = mSmartDashboardInteractions.getJoystickControlsMode();
            
            // Reset drive
            drive.resetEncoders();

           // Configure loopList
            loopList.start();
            
            drive.setOpenLoop(DriveSignal.NEUTRAL);
            drive.setBrakeMode(false);

      		dataLogger.putNumber("OperationalMode", OperationalMode.TELEOP.getVal());
    		DataLogger.setOutputMode(DataLogger.OutputMode.SMARTDASHBOARD_AND_FILE);
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
            drive.setOpenLoop( controls.getDriveSignal() );
            
            log();           		
    	}
    	catch (Throwable t) 
    	{
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
   	
    } 
}