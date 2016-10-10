
package org.usfirst.frc.team686.simsbot;

import java.io.File;
import java.io.IOException;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import org.usfirst.frc.team686.lib.util.DriveSignal;

import org.usfirst.frc.team686.simsbot.JoystickControls;
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
	PowerDistributionPanel pdp;
	
	Drive drive = Drive.getInstance();
	JoystickControls controls = JoystickControls.getInstance();
	
	DataLogger dataLogger = DataLogger.getInstance();
    

	
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
		// TODO: try/catch CrashTracker
		
        System.out.println("Running SimsBot robotInit()");

       // Reset all state
        zeroAllSensors();
        

    	// Set dataLogger and Time information
    	TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));

    	// Determine folder for log files
    	findLogDirectory();
    	
    	// add signals to logger
    	populateDataLogger();
    }
    
	
	public void findLogDirectory()
	{

		// Determine folder for log files
		File logDirectory = null;
		if (logDirectory == null) logDirectory = checkLogDirectory(new File("/dev/sda1"));
		if (logDirectory == null)
		{
			logDirectory = new File("/home/admin/logs");
		    if (!logDirectory.exists())
		    {
			    logDirectory.mkdir();
		    }
		}
		if (logDirectory != null && logDirectory.isDirectory())
		{
			String logMessage = String.format("Log directory is %s\n", logDirectory);
			System.out.print (logMessage);
			dataLogger.setDirectory(logDirectory);
			dataLogger.setMinimumInterval(1000);
		}	        
	}
	
	public File checkLogDirectory (File root)
	{
		// does the root directory exist?
		if (!root.isDirectory()) return null;
		
		File logDirectory = new File(root, "logs");
		if (!logDirectory.isDirectory()) return null;
		
		return logDirectory;
	}
	
	public void populateDataLogger()
	{
		if (dataLogger.shouldLogData())
		{
			dataLogger.addDataItem("lMotorCurrent",  pdp.getCurrent(15));
			dataLogger.addDataItem("rMotorCurrent",  pdp.getCurrent(0));
			dataLogger.addDataItem("lMotorCtrl",     drive.leftMotor_.get());
			dataLogger.addDataItem("rMotorCtrl",  	 drive.rightMotor_.get());
		}
	}	
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    @Override
    public void autonomousInit() 
    {
        System.out.println("Running SimsBot autonomousInit()");
    	
        // Reset all state
        zeroAllSensors();
    }

    private boolean autoFirstRun = true;
    
    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() 
    {
        if (autoFirstRun) 
        {
            System.out.println("Running SimsBot autoPeriodic()");
            autoFirstRun = false;

        }
     }

    @Override
    public void teleopInit()
    {
        System.out.println("Running SimsBot teleopInit()");
    	
         // Reset drive encoders
        drive.resetEncoders();
        
        drive.setOpenLoop(DriveSignal.NEUTRAL);
    	drive.setBrakeMode(false);
    }
    
    
    private boolean teleFirstRun = true;
    
    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() 
    {
        if (teleFirstRun) 
        {
            System.out.println("Running SimsBot teleopPeriodic()");
            teleFirstRun = false;
        }
    	
        drive.setOpenLoop(drive.tankDrive( controls.getThrottle(), controls.getTurn() ));
    }
        
    
    /**
     * This function is called periodically during test mode
     */
    @Override
    public void testPeriodic()
    {
    
    }
    
    private boolean dpFirstRun = true;
    
    @Override
    public void disabledPeriodic() 
    {
        if (dpFirstRun) 
        {
            System.out.println("Running SimsBot disabledPeriodic()");
            dpFirstRun = false;
        }
        drive.stop();
    }

 
    
}
