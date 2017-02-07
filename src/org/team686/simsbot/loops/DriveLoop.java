package org.team686.simsbot.loops;

import com.ctre.CANTalon;
import org.team686.lib.sensors.BNO055;
import org.team686.simsbot.Constants;
import org.team686.simsbot.subsystems.Drive;
import org.team686.simsbot.subsystems.Drive.DriveControlState;


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
    
    DriveControlState driveControlState;
    
	public final CANTalon lMotor, rMotor;

	private static final int kVelocityControlSlot = 0;
	private static final int kBaseLockControlSlot = 1;
	private boolean brakeEnable = true;

	
	
	private DriveLoop() 
	{
		lMotor = new CANTalon(Constants.kLeftMotorTalonId);
		rMotor = new CANTalon(Constants.kRightMotorTalonId);

		// Get status at 100Hz
		lMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

		// Start in open loop mode
		lMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		rMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		setLeftRightPower(0, 0);
		setBrakeMode(false);

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
		synchronized(drive)	// don't allow changes to drive until we finish updating Talon SRXs
		{
			// get encoder values from hardware, set in Drive
			drive.setLeftDistanceInches(  rotationsToInches( lMotor.getPosition() ));
			drive.setRightDistanceInches( rotationsToInches( rMotor.getPosition() ));
	
			drive.setLeftSpeedInchesPerSec(  rpmToInchesPerSecond( lMotor.getSpeed() ));
			drive.setRightSpeedInchesPerSec( rpmToInchesPerSecond( rMotor.getSpeed() ));
	
			/*
			 * measured angle decreases with clockwise rotation
			 * it should increase with clockwise rotation (according to
			 * documentation, and standard right hand rule convention
			 * negate it here to correct
			 */
			drive.setHeadingDeg( -imu.getHeading() );

			drive.setMotorCurrent(lMotor.getOutputCurrent(), rMotor.getOutputCurrent() );
			drive.setMotorCtrl(lMotor.get(), rMotor.get() );
			drive.setMotorPIDError(lMotor.getClosedLoopError(), rMotor.getClosedLoopError() );
			
			
			// get Drive commands and send to hardware
			resetEncoders();
			setControlState();
			updateMotors();
			setBrakeMode();
		}
	}

	@Override public void onStop()
	{
		stopMotors();
	}

	private void stopMotors()
	{
		driveControlState = DriveControlState.OPEN_LOOP;
		setControlState();
		setLeftRightPower(0,0); 
	}
	
	
	private static double rotationsToInches(double rotations) {	return rotations * Constants.kDriveWheelCircumInches; }
	private static double inchesToRotations(double inches) { return inches / Constants.kDriveWheelCircumInches; }

	private static double rpmToInchesPerSecond(double rpm) { return rotationsToInches(rpm) / 60; }
	private static double inchesPerSecondToRpm(double inches_per_second) { return inchesToRotations(inches_per_second) * 60; }

	
	private void setControlState()
    {
		DriveControlState newState = drive.getControlState();
		
		if (newState != driveControlState)
		{
	        switch (newState)
	        {
	        	case OPEN_LOOP: 
	                lMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	                rMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    			setBrakeMode(false);
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
	
	    			setBrakeMode(true);
	        		break;
	        		
	        	case VELOCITY_SETPOINT:
	        		configureTalonsForSpeedControl();
	    			setBrakeMode(false);
	        		break;
	        		
	        	case VELOCITY_HEADING_CONTROL:
	        		drive.resetVelocityHeadingPID();
	    			configureTalonsForSpeedControl();
	    			setBrakeMode(false);
	        		break;
	        		
	        	case PATH_FOLLOWING_CONTROL:
	        		drive.resetVelocityHeadingPID();
	    			configureTalonsForSpeedControl();
	    			setBrakeMode(false);
	        		break;
	        		
	        	default:
	        		break;
	        }
	        // set new state
			driveControlState = newState;
		}
	}

	
	
	private void updateMotors()
    {
		double lMotorCtrl = drive.getLeftMotorCtrl();
		double rMotorCtrl = drive.getRightMotorCtrl();
		
        switch (driveControlState)
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

	private void setBrakeMode()
	{
		setBrakeMode( drive.getBrakeEnableCmd() );
	}
	
	
	private void setBrakeMode(boolean val) 
	{
		if (brakeEnable != val) 
		{
			lMotor.enableBrakeMode(val);
			rMotor.enableBrakeMode(val);
			brakeEnable = val;
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

	
	private void setLeftRightPower(double left, double right) 
	{
		lMotor.set(left);
		rMotor.set(right);
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
