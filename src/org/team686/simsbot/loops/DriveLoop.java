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
    
	public final CANTalon lMotor, rMotor;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	
	private DriveLoop() 
	{
		drive = Drive.getInstance();
		imu = BNO055.getInstance(Constants.BNO055_PORT);
		driveStatus = DriveStatus.getInstance();
		
		lMotor = new CANTalon(Constants.kLeftMotorTalonId);
		rMotor = new CANTalon(Constants.kRightMotorTalonId);

		// Get status at 100Hz
		lMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

		// Set initial settings
		DriveCommand neutralCmd = DriveCommand.NEUTRAL();
		setControlMode(neutralCmd);
		setMotors(neutralCmd);
		setBrakeMode(neutralCmd);
		resetEncoders(neutralCmd);
		
        lMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		lMotor.set(0);
		rMotor.set(0);
		lMotor.enableBrakeMode(false);
		rMotor.enableBrakeMode(false);

		// Set up the encoders
		lMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lMotor.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);	// using this API lets us program velocity in RPM in closed-loop modes
		rMotor.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);	// Talon SRX Software Reference Manual Section 17.2 API Unit Scaling
		lMotor.setInverted(false);
		rMotor.setInverted(false);
		lMotor.reverseSensor(false);
		rMotor.reverseSensor(true); // inverts feedback in closed loop modes
		lMotor.reverseOutput(false);
		rMotor.reverseOutput(false);

		// Load velocity control gains
		lMotor.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		rMotor.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		// Load base lock control gains
		lMotor.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);
		rMotor.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
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
			driveStatus.setTalonControlMode( lMotor.getControlMode() );
			driveStatus.setBrakeMode( lMotor.getBrakeEnableDuringNeutral() );
			
			// get encoder values from hardware, set in Drive
			driveStatus.setLeftDistanceInches(  rotationsToInches( lMotor.getPosition() ));
			driveStatus.setRightDistanceInches( rotationsToInches( rMotor.getPosition() ));
	
			driveStatus.setLeftSpeedInchesPerSec(  rpmToInchesPerSecond( lMotor.getSpeed() ));
			driveStatus.setRightSpeedInchesPerSec( rpmToInchesPerSecond( rMotor.getSpeed() ));
	
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			driveStatus.setHeadingDeg( -imu.getHeading() );
	
			driveStatus.setMotorCurrent(lMotor.getOutputCurrent(), rMotor.getOutputCurrent() );
			driveStatus.setMotorStatus(lMotor.get(), rMotor.get() );
			driveStatus.setMotorPIDError(lMotor.getClosedLoopError(), rMotor.getClosedLoopError() );
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
            lMotor.changeControlMode(newMode);
            rMotor.changeControlMode(newMode);
			
	        switch (newMode)
	        {
	        	case PercentVbus: 
	                break;
	
	        	case Position:
	    			lMotor.setProfile(kBaseLockControlSlot);
	    			rMotor.setProfile(kBaseLockControlSlot);

	    			lMotor.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	    			rMotor.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);

	        		lMotor.set(lMotor.getPosition());
	        		rMotor.set(rMotor.getPosition());
	        		break;
	        		
	        	case Speed:
	        		lMotor.setProfile(kVelocityControlSlot);
	        		rMotor.setProfile(kVelocityControlSlot);
	        		
	        		lMotor.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	        		rMotor.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
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
			lMotor.enableBrakeMode(newBrake);
			rMotor.enableBrakeMode(newBrake);
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
        		lMotor.set(lMotorCtrl);
        		rMotor.set(rMotorCtrl);
        		break;

        	case Position:
        		// initial position already set on mode change
        		break;
        		
        	case Speed:
        		// DriveCommand given in inches/sec
        		// Talon SRX needs RPM in closed-loop mode.
        		// convert inches/sec to RPM
           		lMotor.set(inchesPerSecondToRpm(lMotorCtrl)); 
        		rMotor.set(inchesPerSecondToRpm(rMotorCtrl));
        		break;
        		
        	case Disabled:
        	default:
        		lMotor.set(0);
        		rMotor.set(0);
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
			lMotor.setPosition(0);
			rMotor.setPosition(0);
				
			lMotor.setEncPosition(0);
			rMotor.setEncPosition(0);
			
			// cannot reset gyro heading in hardware.  
			// calibration to desired initial pose is done in RobotState.reset() called from Robot.autonomousInit()  
		}
	}	


};
