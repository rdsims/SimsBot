
package org.team686.simsbot;

import java.util.TimeZone;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.joystick.*;
import org.team686.lib.util.*;

import org.team686.simsbot.auto.AutoModeExecuter;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.command_status.DriveStatus;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.command_status.VisionStatus;
import org.team686.simsbot.loops.DriveLoop;
import org.team686.simsbot.loops.LoopController;
import org.team686.simsbot.loops.RobotStateLoop;
import org.team686.simsbot.loops.VisionLoop;
import org.team686.simsbot.subsystems.Drive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	PowerDistributionPanel pdp = new PowerDistributionPanel();	// TODO: add relay for LED light ring

	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
	RobotState robotState = RobotState.getInstance();	
	Drive drive = Drive.getInstance();					
	
	AutoModeExecuter autoModeExecuter = null;

	LoopController loopController;
	
	SmartDashboardInteractions smartDashboardInteractions;
	DataLogController robotLogger;	// logger for Robot thread (autonomous thread has it's own logger)

	enum OperationalMode 
	{
		DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);

		private int val;

		private OperationalMode(int val) { this.val = val; }
		public int getVal() { return val; }
	}

	OperationalMode operationalMode = OperationalMode.DISABLED;
	
	public Robot() {
		CrashTracker.logRobotConstruction();
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

			// Configure LoopController
			loopController = new LoopController();
			loopController.register(drive.getVelocityPIDLoop());
			loopController.register(DriveLoop.getInstance());
			loopController.register(RobotStateLoop.getInstance());
			loopController.register(VisionLoop.getInstance());

			smartDashboardInteractions = new SmartDashboardInteractions();
			smartDashboardInteractions.initWithDefaults();

			// Set dataLogger and Time information
			TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));

			robotLogger = DataLogController.getRobotLogController();
			robotLogger.register(this.getLogger());
			robotLogger.register(Drive.getInstance().getLogger());
			robotLogger.register(drive.getCommand().getLogger());
			robotLogger.register(DriveStatus.getInstance().getLogger());
			robotLogger.register(RobotState.getInstance().getLogger());
			robotLogger.register(VisionStatus.getInstance().getLogger());
			
			// set initial Pose (will be updated during autonomousInit())
			setInitialPose( new Pose() );
			
		} 
		catch (Throwable t) 
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public void setInitialPose( Pose _initialPose )
	{
		zeroAllSensors();											// make sure all encoders are zeroed out
		robotState.reset(Timer.getFPGATimestamp(), DriveStatus.getInstance().getLeftDistanceInches(), DriveStatus.getInstance().getRightDistanceInches(), _initialPose);	// set initial pose
		System.out.println("InitialPose: " + _initialPose);
	}
	
	public void zeroAllSensors() 
	{
		drive.zeroSensors();
		// mSuperstructure.zeroSensors();
	}
	
	public void stopAll() 
	{
		drive.stop();
		// mSuperstructure.stop();
	}

	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	@Override
	public void disabledInit() 
	{
		operationalMode = OperationalMode.DISABLED;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);
		
		try 
		{
			CrashTracker.logDisabledInit();
			if (autoModeExecuter != null) 
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			stopAll(); 			// stop all actuators
			zeroAllSensors();	// keep sensors in reset, as robot is moved onto field

			// keep loopController running until stopAll() and zeroAllSensors() have been processed
			double startTime = Timer.getFPGATimestamp();
			double elapsedTime = 0;
			while (drive.getCommand().getResetEncoders() || elapsedTime < 0.5)
			{
				// watchdog timer
				elapsedTime = Timer.getFPGATimestamp() - startTime;
			}
			loopController.stop();	
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
			stopAll(); 			// stop all actuators
			zeroAllSensors();	// keep sensors in reset, as robot is moved onto field

			System.gc(); // runs garbage collector
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
		operationalMode = OperationalMode.AUTONOMOUS;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		try 
		{
			CrashTracker.logAutoInit();
			if (autoModeExecuter != null) 
			{
				autoModeExecuter.stop();
			}
			autoModeExecuter = null;

			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getAutoModeSelection());

			setInitialPose( autoModeExecuter.getAutoMode().getInitialPose() );		

			loopController.start();
			autoModeExecuter.start();

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
			// outputAllToSmartDashboard();
			// updateDriverFeedback();
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
		operationalMode = OperationalMode.TELEOP;
		boolean logToFile = true;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		try 
		{
			CrashTracker.logTeleopInit();

			// Select joystick control method
			controls = smartDashboardInteractions.getJoystickControlsMode();

			// Configure looper
			loopController.start();

			drive.setOpenLoop(DriveCommand.NEUTRAL());

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
			drive.setOpenLoop(controls.getDriveCommand());
		} 
		catch (Throwable t) 
		{
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}
	
	
	
	@Override
	public void testInit() 
	{
		loopController.start();
	}

	@Override
	public void testPeriodic()
	{
		drive.testDriveSpeedControl();
	}
	
	
	// called after disabledPeriodic, autoPeriodic, and teleopPeriodic 
	@Override
	public void robotPeriodic()
	{
		robotLogger.log();
	}


	
	
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("OperationalMode", operationalMode.getVal());
        }
    };
    
    public DataLogger getLogger() { return logger; }
	
	
}