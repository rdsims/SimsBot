package org.team686.simsbot.loops;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.sensors.BNO055;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.command_status.DriveStatus;
import org.team686.simsbot.subsystems.Drive;

/*
 * DriveLoop is the interface between Drive.java and the actual hardware.
 * It runs periodically, taking the commands sent by Drive and sending them to the hardware.
 * In this way, Drive.java does not access the hardware directly.  The benefits of this partition are: 
 * 1) Changes to drive hardware only requires changes to DriveLoop, not Drive
 * 2) DriveLoop can be easily replaced for simulation purposes.
 */

public class DriveLoop implements Loop 
{
	private static DriveLoop instance = new DriveLoop();
	public static DriveLoop getInstance() { return instance; }
	
    private static Drive drive;
	private static BNO055 imu;
    private DriveStatus driveStatus;
    
	public final CANTalon lMotorMaster, lMotorSlave;
	public final CANTalon rMotorMaster, rMotorSlave;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	
	private DriveLoop() 
	{
		drive = Drive.getInstance();
		imu = BNO055.getInstance(Constants.BNO055_PORT);
		driveStatus = DriveStatus.getInstance();
		
		lMotorMaster = new CANTalon(Constants.kLeftMotorMasterTalonId);
        lMotorSlave  = new CANTalon(Constants.kLeftMotorSlaveTalonId);

		rMotorMaster = new CANTalon(Constants.kRightMotorMasterTalonId);
        rMotorSlave  = new CANTalon(Constants.kRightMotorSlaveTalonId);

		lMotorSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rMotorSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		lMotorSlave.set(Constants.kLeftMotorMasterTalonId);		// give slave the TalonID of it's master
		rMotorSlave.set(Constants.kRightMotorMasterTalonId);	// give slave the TalonID of it's master
        
		// Get status at 100Hz
		lMotorMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rMotorMaster.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

		// Set initial settings
		DriveCommand neutralCmd = DriveCommand.NEUTRAL();
		setControlMode(neutralCmd);
		setMotors(neutralCmd);
		setBrakeMode(neutralCmd);
		resetEncoders(neutralCmd);
		
        lMotorMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rMotorMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		lMotorMaster.set(0);
		rMotorMaster.set(0);
		lMotorMaster.enableBrakeMode(false);
		rMotorMaster.enableBrakeMode(false);
		lMotorSlave.enableBrakeMode(false);
		rMotorSlave.enableBrakeMode(false);

		// Set up the encoders
		lMotorMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rMotorMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lMotorMaster.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);	// using this API lets us program velocity in RPM in closed-loop modes
		rMotorMaster.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);	// Talon SRX Software Reference Manual Section 17.2 API Unit Scaling
		lMotorMaster.setInverted(false);
		rMotorMaster.setInverted(true);		// right motor controls are reversed
		lMotorMaster.reverseSensor(false);
		rMotorMaster.reverseSensor(false); // inverts feedback in closed loop modes
		lMotorMaster.reverseOutput(false);
		rMotorMaster.reverseOutput(false);
		lMotorSlave.reverseOutput(false);
		rMotorSlave.reverseOutput(false);

		// Load velocity control gains
		lMotorMaster.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		rMotorMaster.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		// Load base lock control gains
		lMotorMaster.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);
		rMotorMaster.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);
	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		// get status from hardware
		getStatus();
		
		// send new commands to hardware
		sendCommands();
	}

	@Override public void onStop()
	{
		stopMotors();
	}

	private void stopMotors()
	{
		drive.setCommand(DriveCommand.NEUTRAL());		// override any incoming commands
		sendCommands();
	}

	private void getStatus()
	{
		synchronized(driveStatus)	// lock DriveStatus until we update it, so that objects reading DriveStatus don't get partial updates	
		{
			// get Talon control & brake modes (assume right motor is configured identically)
			driveStatus.setTalonControlMode( lMotorMaster.getControlMode() );
			driveStatus.setBrakeMode( lMotorMaster.getBrakeEnableDuringNeutral() );
			
			// get encoder values from hardware, set in Drive
			driveStatus.setLeftDistanceInches(  rotationsToInches( lMotorMaster.getPosition() ));
			driveStatus.setRightDistanceInches( rotationsToInches( rMotorMaster.getPosition() ));
	
			driveStatus.setLeftSpeedInchesPerSec(  rpmToInchesPerSecond( lMotorMaster.getSpeed() ));
			driveStatus.setRightSpeedInchesPerSec( rpmToInchesPerSecond( rMotorMaster.getSpeed() ));
	
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			driveStatus.setHeadingDeg( -imu.getHeading() );
	
			driveStatus.setMotorCurrent(lMotorMaster.getOutputCurrent(), rMotorMaster.getOutputCurrent() );
			driveStatus.setMotorStatus(lMotorMaster.get(), rMotorMaster.get() );
			driveStatus.setMotorPIDError(lMotorMaster.getClosedLoopError(), rMotorMaster.getClosedLoopError() );
		}
	}
		
	private void sendCommands()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// Watchdog timer  
		double currentTime = Timer.getFPGATimestamp();
		if (currentTime - newCmd.getCommandTime() > Constants.kDriveWatchdogTimerThreshold)
		{
			// Halt robot if new command hasn't been sent in a while
			stopMotors();
			return;
		}
				
		synchronized(newCmd)	// lock DriveCommand so no one changes it under us while we are sending the commands
		{
			setControlMode(newCmd);
			setMotors(newCmd);
			setBrakeMode(newCmd);
			resetEncoders(newCmd);
		}
	}
	
	
	private void setControlMode(DriveCommand newCmd)
    {
		TalonControlMode newMode = newCmd.getTalonControlMode();
		
		if (newMode != driveStatus.getTalonControlMode())
		{
            lMotorMaster.changeControlMode(newMode);
            rMotorMaster.changeControlMode(newMode);
			
	        switch (newMode)
	        {
	        	case PercentVbus: 
	                break;
	
	        	case Position:
	    			lMotorMaster.setProfile(kBaseLockControlSlot);
	    			rMotorMaster.setProfile(kBaseLockControlSlot);

	    			lMotorMaster.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	    			rMotorMaster.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);

	        		lMotorMaster.set(lMotorMaster.getPosition());
	        		rMotorMaster.set(rMotorMaster.getPosition());
	        		break;
	        		
	        	case Speed:
	        		lMotorMaster.setProfile(kVelocityControlSlot);
	        		rMotorMaster.setProfile(kVelocityControlSlot);
	        		
	        		lMotorMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	        		rMotorMaster.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	        		break;
	        		
	        	case Disabled:
	        	default:
	        		break;
	        }
		}
	}
	
	
	
	private void setBrakeMode(DriveCommand newCmd)
	{
		boolean newBrake = newCmd.getBrake();
		setBrakeMode(newBrake);
	}
	
	
	private void setBrakeMode(boolean newBrake) 
	{
		if (newBrake != driveStatus.getBrakeMode()) 
		{
			lMotorMaster.enableBrakeMode(newBrake);
			rMotorMaster.enableBrakeMode(newBrake);
			lMotorSlave.enableBrakeMode(newBrake);
			rMotorSlave.enableBrakeMode(newBrake);
		}
	}
	
	
		
	private void setMotors(DriveCommand newCmd)
    {
		double lMotorCtrl = newCmd.getLeftMotor();
		double rMotorCtrl = newCmd.getRightMotor();
		
        switch (newCmd.getTalonControlMode())	// assuming new mode is already configured
        {
        	case PercentVbus:
        		// DriveCommand given in range +/-1, with 1 representing full throttle
        		lMotorMaster.set(lMotorCtrl);
        		rMotorMaster.set(rMotorCtrl);
        		break;

        	case Position:
        		// initial position already set on mode change
        		break;
        		
        	case Speed:
        		// DriveCommand given in inches/sec
        		// Talon SRX needs RPM in closed-loop mode.
        		// convert inches/sec to RPM
           		lMotorMaster.set(inchesPerSecondToRpm(lMotorCtrl)); 
        		rMotorMaster.set(inchesPerSecondToRpm(rMotorCtrl));
        		break;
        		
        	case Disabled:
        	default:
        		lMotorMaster.set(0);
        		rMotorMaster.set(0);
        		break;
        }
	}

	// Talon SRX reports position in rotations while in closed-loop Position mode
	private static double rotationsToInches(double _rotations) {	return _rotations * Constants.kDriveWheelCircumInches; }
	private static double inchesToRotations(double _inches) { return _inches / Constants.kDriveWheelCircumInches; }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	private static double rpmToInchesPerSecond(double _rpm) { return rotationsToInches(_rpm) / 60.0; }
	private static double inchesPerSecondToRpm(double _inches_per_second) { return inchesToRotations(_inches_per_second) * 60.0; }

	
	

	private void resetEncoders(DriveCommand newCmd)
	{
		if (newCmd.getResetEncoders())
		{
			lMotorMaster.setPosition(0);
			rMotorMaster.setPosition(0);
				
			lMotorMaster.setEncPosition(0);
			rMotorMaster.setEncPosition(0);
			
			// cannot reset gyro heading in hardware.  
			// calibration to desired initial pose is done in RobotState.reset() called from Robot.autonomousInit()  
		}
	}	


};
