
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
public class Robot extends IterativeRobot {
	PowerDistributionPanel pdp = new PowerDistributionPanel();

	Drive drive = Drive.getInstance();
	AutoModeExecuter mAutoModeExecuter = null;

	JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
	DataLogger dataLogger = DataLogger.getInstance();
	//DataLogger autonomousLogger = DataLogger.getAutonomousInstance();
	DataLogger visionLogger = DataLogger.getVisionInstance();
	RobotState mRobotState = RobotState.getInstance();

	LoopController loopController = new LoopController();

	SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();

	enum OperationalMode {
		DISABLED(0), AUTONOMOUS(1), TELEOP(2), TEST(3);

		private int val;

		private OperationalMode(int val) {
			this.val = val;
		}

		public int getVal() {
			return val;
		}
	}

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
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			// Reset all state
			zeroAllSensors();

			// Configure LoopController
			loopController.register(drive.getVelocityPIDLoop());
			loopController.register(VisionLoop.getInstance());
			loopController.register(RobotStateLoop.getInstance());

			// Set dataLogger and Time information
			TimeZone.setDefault(TimeZone.getTimeZone("America/New_York"));

			mSmartDashboardInteractions.initWithDefaults();

			// Determine folder for log files
			DataLogger.findLogDirectory();
			// set data logger file bases
			dataLogger.setFileBase("main");
			//autonomousLogger.setFileBase("auto");
			visionLogger.setFileBase("vision");
			
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public void stopAll() {
		drive.stop();
		// mSuperstructure.stop();
	}

	public void log() {
		drive.log();
		mRobotState.log();
		loopController.log();
		dataLogger.saveDataItems();
		//autonomousLogger.saveDataItems();
		visionLogger.saveDataItems();
	}

	/****************************************************************
	 * DISABLED MODE
	 ****************************************************************/

	@Override
	public void disabledInit() {
		try {
			CrashTracker.logDisabledInit();
			if (mAutoModeExecuter != null) {
				mAutoModeExecuter.stop();
			}
			mAutoModeExecuter = null;

			loopController.stop();

			drive.setOpenLoop(DriveCommand.BRAKE);

			stopAll(); // Stop all actuators

			dataLogger.putNumber("OperationalMode", OperationalMode.DISABLED.getVal());
			DataLogger.setOutputMode(DataLogger.OutputMode.SMARTDASHBOARD_ONLY);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledPeriodic() {
		try {
			stopAll();
			drive.resetEncoders();
			log();

			System.gc(); // runs garbage collector
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/****************************************************************
	 * AUTONOMOUS MODE
	 ****************************************************************/

	@Override
	public void autonomousInit() {
		try {
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

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {
			log();
			// outputAllToSmartDashboard();
			// updateDriverFeedback();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/****************************************************************
	 * TELEOP MODE
	 ****************************************************************/

	@Override
	public void teleopInit() {
		try {
			CrashTracker.logTeleopInit();

			// Select joystick control method
			controls = mSmartDashboardInteractions.getJoystickControlsMode();

			// Reset drive
			drive.resetEncoders();

			// Configure looper
			loopController.start();

			drive.setOpenLoop(DriveCommand.NEUTRAL);

			dataLogger.putNumber("OperationalMode", OperationalMode.TELEOP.getVal());
			DataLogger.setOutputMode(DataLogger.OutputMode.SMARTDASHBOARD_AND_FILE);
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	@Override
	public void teleopPeriodic() {
		try {
			drive.setOpenLoop(controls.getDriveCommand());

			log();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}
	
	// called after disabledPeriodic, autoPeriodic, and teleopPeriodic 
	@Override
	public void robotPeriodic() {}

}