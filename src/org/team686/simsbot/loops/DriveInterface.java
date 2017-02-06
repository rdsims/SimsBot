package org.team686.simsbot.loops;

import org.team686.lib.sensors.BNO055;
import org.team686.lib.util.DriveSignal;
import org.team686.lib.util.Rotation2d;
import org.team686.lib.util.SynchronousPID;
import org.team686.simsbot.Constants;
import org.team686.simsbot.subsystems.Drive;
import org.team686.simsbot.subsystems.Drive.DriveControlState;
import org.team686.simsbot.subsystems.Drive.VelocityHeadingSetpoint;

import com.ctre.CANTalon;

public class DriveInterface implements Loop 
{
	private static DriveInterface instance = new DriveInterface();
	public static DriveInterface getInstance() { return instance; }
	
    static Drive drive = Drive.getInstance();
    
    DriveControlState driveControlState;
    
	public final CANTalon lMotor_, rMotor_;
	double lDistance, rDistance, lSpeed, rSpeed;
	
	protected static final int kVelocityControlSlot = 0;
	protected static final int kBaseLockControlSlot = 1;
	private boolean isBrakeMode_ = true;

	
	private DriveInterface() 
	{
		lMotor_ = new CANTalon(Constants.kLeftMotorTalonId);
		rMotor_ = new CANTalon(Constants.kRightMotorTalonId);

		// Get status at 100Hz
		lMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
		rMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

		// Start in open loop mode
		lMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		rMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		setLeftRightPower(0, 0);
		setBrakeMode(false);

		// Set up the encoders
		lMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		lMotor_.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);
		lMotor_.setInverted(false);
		lMotor_.reverseSensor(false);
		lMotor_.reverseOutput(false);

		rMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		rMotor_.configEncoderCodesPerRev(Constants.kQuadEncoderCodesPerRev);
		rMotor_.setInverted(false);
		rMotor_.reverseSensor(true); // inverts feedback in closed loop modes
		rMotor_.reverseOutput(false);

		// Load velocity control gains
		lMotor_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		rMotor_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
				Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
				kVelocityControlSlot);
		// Load base lock control gains
		lMotor_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);
		rMotor_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
				Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
				kBaseLockControlSlot);

		setOpenLoop(DriveSignal.NEUTRAL);
	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		lDistance = lMotor_.getPosition();
		rDistance = rMotor_.getPosition();
		lSpeed = lMotor_.getSpeed();
		rSpeed = rMotor_.getSpeed();
		
		synchronized(drive)	// don't allow changes to drive until we finish updating Talon SRXs
		{
			setControlState(drive.getControlState());
			update(drive.getLeftMotorCtrl(), drive.getRightMotorCtrl());
		}
	}

	@Override public void onStop()
	{
		setLeftRightPower(0, 0);
	}

	public void setControlState(DriveControlState newState)
    {
		if (newState != driveControlState)
		{
	        switch (newState)
	        {
	        	case OPEN_LOOP: 
	                lMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	                rMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	                break;
	
	        	case BASE_LOCKED:
	    			lMotor_.setProfile(kBaseLockControlSlot);
	    			lMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
	    			lMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	    			lMotor_.set(lMotor_.getPosition());
	
	    			rMotor_.setProfile(kBaseLockControlSlot);
	    			rMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
	    			rMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
	    			rMotor_.set(rMotor_.getPosition());
	
	    			setBrakeMode(true);
	        		break;
	        		
	        	case VELOCITY_SETPOINT:
	        		configureTalonsForSpeedControl();
	        		break;
	        		
	        	case VELOCITY_HEADING_CONTROL:
	    			configureTalonsForSpeedControl();
	    			velocityHeadingPid_.reset();
	        		break;
	        		
	        	case PATH_FOLLOWING_CONTROL:
	    			configureTalonsForSpeedControl();
	    			velocityHeadingPid_.reset();
	        		break;
	        		
	        	default:
	        		break;
	        }
	        // set new state
			driveControlState = newState;
		}
	}

	
	
	public void update(double lMotorCtrl, double rMotorCtrl)
    {
        switch (driveControlState)
        {
        	case OPEN_LOOP: 
        	case VELOCITY_SETPOINT:
        	case VELOCITY_HEADING_CONTROL:
        	case PATH_FOLLOWING_CONTROL:
    			setLeftRightPower(lMotorCtrl, lMotorCtrl);
        		break;

        	case BASE_LOCKED:
        		// no changes
        		break;
        		
        	default:
        		break;
        }
	}

	
	public void setBrakeMode(boolean val) 
	{
		if (isBrakeMode_ != val) 
		{
			lMotor_.enableBrakeMode(val);
			rMotor_.enableBrakeMode(val);
			isBrakeMode_ = val;
		}
	}
	
	
	private void configureTalonsForSpeedControl()
	{
		lMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
		lMotor_.setProfile(kVelocityControlSlot);
		lMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);

		rMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
		rMotor_.setProfile(kVelocityControlSlot);
		rMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
	}

	
	protected synchronized void setLeftRightPower(double left, double right) 
	{
		lMotor_.set(left);
		rMotor_.set(right);
	}

	
	public synchronized void resetEncoders()
	{
		lMotor_.setPosition(0);
		rMotor_.setPosition(0);

		lMotor_.setEncPosition(0);
		rMotor_.setEncPosition(0);
	}

	
	public double getLeftDistance()  { return lDistance; }
	public double getRightDistance() { return rDistance; }
	public double getLeftSpeed()     { return lSpeed; }
	public double getRightSpeed()    { return rSpeed; }

	
};
