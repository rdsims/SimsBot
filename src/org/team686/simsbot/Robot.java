
package org.team686.simsbot;

import java.io.File;
import java.io.IOException;
import java.util.TimeZone;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team686.lib.joystick.*;
import org.team686.lib.util.*;

import org.team686.simsbot.auto.AutoModeExecuter;
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
	PowerDistributionPanel pdp = new PowerDistributionPanel();

	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
	RobotState mRobotState = RobotState.getInstance();	
	Drive drive = Drive.getInstance();					
	
	AutoModeExecuter mAutoModeExecuter = null;

	// instantiate loops after Drive, RobotState
	LoopController loopController = new LoopController();
	
	
	SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();
	DataLogController robotLogger = new DataLogController();	// logger for Robot thread (autonomous thread has it's own logger)

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

			// Configure LoopController
			loopController.register(drive.getVelocityPIDLoop());
			loopController.register(DriveLoop.getInstance());
			loopController.register(RobotStateLoop.getInstance());
			loopController.register(VisionLoop.getInstance());

			// Set dataLogger and Time information
			TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));

			mSmartDashboardInteractions.initWithDefaults();

			// Determine folder for log files
			DataLogController.findLogDirectory();
			
			// set data logger file base
			robotLogger.setFileBase("robot");
			
			robotLogger.register(this.getLogger());
			robotLogger.register(Drive.getInstance().getLogger());
			robotLogger.register(DriveStatus.getInstance().getLogger());
			robotLogger.register(RobotState.getInstance().getLogger());
			
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
		// mSuperstructure.stop();
	}

	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	@Override
	public void disabledInit() 
	{
		operationalMode = OperationalMode.DISABLED;
		boolean logToFile = false;
		boolean logToSmartDashboard = true;
		robotLogger.setOutputMode(logToFile, logToSmartDashboard);
		
		try 
		{
			CrashTracker.logDisabledInit();
			if (mAutoModeExecuter != null) 
			{
				mAutoModeExecuter.stop();
			}
			mAutoModeExecuter = null;

			loopController.stop();

			drive.setOpenLoop(DriveCommand.BRAKE);

			stopAll(); // Stop all actuators

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
			if (mAutoModeExecuter != null) 
			{
				mAutoModeExecuter.stop();
			}
			mAutoModeExecuter = null;

			// Reset all sensors
			drive.resetEncoders();
			zeroAllSensors();

			loopController.start();

			mAutoModeExecuter = new AutoModeExecuter();
			mAutoModeExecuter.setAutoMode(mSmartDashboardInteractions.getSelectedAutonMode());

/*			
			AutoModeBase mAutoMode = mAutoModeExecuter.getAutoMode();
			if (mAutoMode instanceof AutoPlacePegMode) {
				Translation2d initialPosition = mAutoMode.getInitialPosition();
				Rotation2d initialHeading = new Rotation2d();
				System.out.println("InitialPosition: (" + initialPosition.getX() + ", " + initialPosition.getY() + ")");
				mRobotState.reset(0.0, new RigidTransform2d(initialPosition, initialHeading), new Rotation2d());
			}
*/
			
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
			controls = mSmartDashboardInteractions.getJoystickControlsMode();

			// Reset drive
			drive.resetEncoders();

			// Configure looper
			loopController.start();

			drive.setOpenLoop(DriveCommand.NEUTRAL);

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
			putNumber("OperationalMode", operationalMode.getVal());
        }
    };
    
    public DataLogger getLogger() { return logger; }
	
	
}