package org.team686.simsbot.loops;

import com.ctre.CANTalon;
import org.team686.lib.sensors.BNO055;
import org.team686.lib.util.DriveCommand;
import org.team686.lib.util.DriveCommand.DriveControlMode;
import org.team686.lib.util.DriveStatus;
import org.team686.simsbot.Constants;
import org.team686.simsbot.subsystems.Drive;

//TODO: update to use DriveSignal consistently

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
	
    private static Drive drive = Drive.getInstance();
	private static BNO055 imu = BNO055.getInstance(Constants.BNO055_PORT);
    
	DriveCommand currentState;
    
	public final CANTalon lMotor, rMotor;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;

	
	private DriveLoop() 
	{
		lMotor = new CANTalon(Constants.kLeftMotorTalonId);
		rMotor = new CANTalon(Constants.kRightMotorTalonId);

		// Get status at 100Hz
		lMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

		// Start in open loop mode
		drive.setCommand(DriveCommand.NEUTRAL);
		sendCommands();

		// Set up the encoders
		lMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lMotor.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);
		lMotor.setInverted(false);
		lMotor.reverseSensor(false);
		lMotor.reverseOutput(false);

		rMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rMotor.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);
		rMotor.setInverted(false);
		rMotor.reverseSensor(true); // inverts feedback in closed loop modes
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

		stopMotors();
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
		drive.setCommand(DriveCommand.NEUTRAL);		// override any incoming commands
		sendCommands();
	}

	private void getStatus()
	{
		synchronized(drive.driveStatus)	// don't allow changes to drive until we finish updating Talon SRXs
		{
			// get encoder values from hardware, set in Drive
			drive.driveStatus.setLeftDistanceInches(  rotationsToInches( lMotor.getPosition() ));
			drive.driveStatus.setRightDistanceInches( rotationsToInches( rMotor.getPosition() ));
	
			drive.driveStatus.setLeftSpeedInchesPerSec(  rpmToInchesPerSecond( lMotor.getSpeed() ));
			drive.driveStatus.setRightSpeedInchesPerSec( rpmToInchesPerSecond( rMotor.getSpeed() ));
	
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			drive.driveStatus.setHeadingDeg( -imu.getHeading() );
	
			drive.driveStatus.setMotorCurrent(lMotor.getOutputCurrent(), rMotor.getOutputCurrent() );
			drive.driveStatus.setMotorStatus(lMotor.get(), rMotor.get() );
			drive.driveStatus.setMotorPIDError(lMotor.getClosedLoopError(), rMotor.getClosedLoopError() );
		}
	}
		
	private void sendCommands()
	{
		resetEncoders();
		
		DriveCommand newCmd = drive.getCommand();
		setControlMode(newCmd);
		setMotors(newCmd);
		setBrake(newCmd);
	}
	
	
	private void setControlMode(DriveCommand newCmd)
    {
		DriveControlMode newMode = newCmd.getMode();
		
		if (newMode != currentState.getMode())
		{
	        switch (newMode)
	        {
	        	case OPEN_LOOP: 
	                lMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	                rMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    			setBrake(false);
	                break;
	
	        	case BASE_LOCKED:
	    			lMotor.setProfile(kBaseLockControlSlot);
	    			lMotor.changeControlMode(CANTalon.TalonControlMode.Position);
	    			lMotor.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	    			lMotor.set(lMotor.getPosition());
	
	    			rMotor.setProfile(kBaseLockControlSlot);
	    			rMotor.changeControlMode(CANTalon.TalonControlMode.Position);
	    			rMotor.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	    			rMotor.set(rMotor.getPosition());
	
	    			setBrake(true);
	        		break;
	        		
	        	case VELOCITY_SETPOINT:
	        		configureTalonsForSpeedControl();
	    			setBrake(false);
	        		break;
	        		
	        	case VELOCITY_HEADING_CONTROL:
	        		drive.resetVelocityHeadingPID();
	    			configureTalonsForSpeedControl();
	    			setBrake(false);
	        		break;
	        		
	        	case PATH_FOLLOWING_CONTROL:
	        		drive.resetVelocityHeadingPID();
	    			configureTalonsForSpeedControl();
	    			setBrake(false);
	        		break;
	        		
	        	default:
	        		break;
	        }
	        // set new state
			currentState.setMode(newMode);
		}
	}


	private void configureTalonsForSpeedControl()
	{
		lMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
		lMotor.setProfile(kVelocityControlSlot);
		lMotor.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);

		rMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
		rMotor.setProfile(kVelocityControlSlot);
		rMotor.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	}

	
	
	
	
	private void setMotors(DriveCommand newCmd)
    {
		double lMotorCtrl = newCmd.getLeftMotor();
		double rMotorCtrl = newCmd.getRightMotor();
		
        switch (newCmd.getMode())	// assuming new mode is already configured
        {
        	case OPEN_LOOP:
        		// l/r motor controls given as % Vbus
    			setLeftRightPower(lMotorCtrl, rMotorCtrl);	 
        		break;

        	case BASE_LOCKED:
        		// no changes
        		break;
        		
        	case VELOCITY_SETPOINT:
        	case VELOCITY_HEADING_CONTROL:
        	case PATH_FOLLOWING_CONTROL:
        		// l/r motor controls given in inches/sec
        		// need to convert to RPM
    			setLeftRightPower(inchesPerSecondToRpm(lMotorCtrl), inchesPerSecondToRpm(rMotorCtrl));
        		break;

        		
        	default:
        		break;
        }
	}

	private void setLeftRightPower(double _left, double _right) 
	{
		lMotor.set(_left);
		rMotor.set(_right);
		currentState.setMotors(_left, _right);
	}

	private static double rotationsToInches(double _rotations) {	return _rotations * Constants.kDriveWheelCircumInches; }
	private static double inchesToRotations(double _inches) { return _inches / Constants.kDriveWheelCircumInches; }

	private static double rpmToInchesPerSecond(double _rpm) { return rotationsToInches(_rpm) / 60.0; }
	private static double inchesPerSecondToRpm(double _inches_per_second) { return inchesToRotations(_inches_per_second) * 60.0; }

	
	

	private void setBrake(DriveCommand newCmd)
	{
		boolean newBrake = newCmd.getBrake();
		setBrake(newBrake);
	}
	
	
	private void setBrake(boolean newBrake) 
	{
		if (newBrake != currentState.getBrake()) 
		{
			lMotor.enableBrakeMode(newBrake);
			rMotor.enableBrakeMode(newBrake);
			currentState.setBrake(newBrake);
		}
	}
	
	
	
	
	private void resetEncoders()
	{
		if (drive.getResetEncoderCmd())
		{
			lMotor.setPosition(0);
			rMotor.setPosition(0);
	
			lMotor.setEncPosition(0);
			rMotor.setEncPosition(0);
			
			//TODO: add gyro
			
			drive.setResetEncoderCmd(false);	// clear flag
		}
	}	


};
