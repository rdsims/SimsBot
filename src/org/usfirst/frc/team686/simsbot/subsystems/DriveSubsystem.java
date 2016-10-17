package org.usfirst.frc.team686.simsbot.subsystems;

//import java.util.Set;

import edu.wpi.first.wpilibj.CANTalon;
//import edu.wpi.first.wpilibj.Counter;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.lib.sensors.BNO055;
import org.usfirst.frc.team686.lib.sensors.BNO055.opmode_t;
import org.usfirst.frc.team686.lib.sensors.BNO055.reg_t;
import org.usfirst.frc.team686.lib.util.DriveSignal;


/**
 * The robot's drivetrain, which implements the Subsystem abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class DriveSubsystem extends Subsystem {
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static DriveSubsystem instance = new DriveSubsystem();

    public static DriveSubsystem getInstance() {
        return instance;
    }

    // The robot drivetrain's various states
    public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
    }

    public final CANTalon lMotor, rMotor;
    private boolean brakeMode = true;
    private DriveControlState driveControlState;

    private final BNO055 imu; 
    
    
    // The constructor instantiates all of the drivetrain components when the
    // robot powers up
    private DriveSubsystem() 
    {
        lMotor = new CANTalon(Constants.kLeftMotorTalonId);
        rMotor = new CANTalon(Constants.kRightMotorTalonId);

        imu = BNO055.getInstance(Constants.BNO055_PORT);

        
        // Get status at 100Hz
        lMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

        // Start in open loop mode
        lMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        lMotor.set(0);
        rMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rMotor.set(0);
        setBrakeMode(false);

        // Set up the encoders
        lMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        /* doesn't work for QuadEncoders
        if (lMotor.isSensorPresent(
                CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }*/
        lMotor.setInverted(false);
        lMotor.reverseSensor(true);
        lMotor.reverseOutput(false);
        rMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        /* doesn't work for QuadEncoders
        if (rMotor.isSensorPresent(CANTalon.FeedbackDevice.QuadEncoder) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }*/
        lMotor.setInverted(true);		// right motor is flipped with respect to left motor.  Need to reverse direction of rotation in PercentVbus mode
        rMotor.reverseSensor(false);	// inverts feedback in closed loop modes
        rMotor.reverseOutput(true);	// reverse direction of rotation in closed loop modes

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

        setOpenLoop(DriveSignal.NEUTRAL);
        /*
    	double currentTime; //seconds
    	double nextTime; //seconds
        
        if (imu.isSensorPresent()) 
        {
	        if (imu.isInitialized())
		    {
		        if (imu.isCalibrated())
			    {
			       	System.out.println("Gyro reporting present, initialized, calibrated");
			    }
			    else
			    {
			       	DriverStation.reportError("Gyro is reporting not calibrated", false);
			    }
		    }
	        else
	        {
	        	DriverStation.reportError("Gyro is reporting not initialized", false);
	        }
        }
        else
        {
            DriverStation.reportError("Could not detect gyro!", false);       	
        }
        */
    }

    /**
     * Limit motor values to the -1.0 to +1.0 range.
     */
    protected static double limit(double num) 
    {
      if (num > 1.0) 
      {
        return 1.0;
      }
      if (num < -1.0) 
      {
        return -1.0;
      }
      return num;
    }    
    
    
    public DriveSignal tankDrive(double throttle, double turn)
    {
	    boolean squaredInputs = true;
	    
	    double moveValue = limit(throttle);
	    double rotateValue = limit(turn);
	    double lMotorSpeed, rMotorSpeed;
	    
	    if (squaredInputs) {
	      // square the inputs (while preserving the sign) to increase fine control
	      // while permitting full power
	      if (moveValue >= 0.0) {
	        moveValue = (moveValue * moveValue);
	      } else {
	        moveValue = -(moveValue * moveValue);
	      }
	      if (rotateValue >= 0.0) {
	        rotateValue = (rotateValue * rotateValue);
	      } else {
	        rotateValue = -(rotateValue * rotateValue);
	      }
	    }
	
	    if (moveValue > 0.0) {
	      if (rotateValue > 0.0) {
	        lMotorSpeed = moveValue - rotateValue;
	        rMotorSpeed = Math.max(moveValue, rotateValue);
	      } else {
	        lMotorSpeed = Math.max(moveValue, -rotateValue);
	        rMotorSpeed = moveValue + rotateValue;
	      }
	    } else {
	      if (rotateValue > 0.0) {
	        lMotorSpeed = -Math.max(-moveValue, rotateValue);
	        rMotorSpeed = moveValue + rotateValue;
	      } else {
	        lMotorSpeed = moveValue - rotateValue;
	        rMotorSpeed = -Math.max(-moveValue, -rotateValue);
	      }
	    }
	    
	    DriveSignal signal = new DriveSignal(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;
    }

    
    
    
    protected synchronized void setLeftRightPower(double left, double right) {
        lMotor.set(left);
        rMotor.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState != DriveControlState.OPEN_LOOP) {
            lMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            driveControlState = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.lMotor, signal.rMotor);
    }

    public synchronized void setBaseLockOn() 
    {
        if (driveControlState != DriveControlState.BASE_LOCKED) 
        {
            lMotor.setProfile(kBaseLockControlSlot);
            lMotor.changeControlMode(CANTalon.TalonControlMode.Position);
            lMotor.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            lMotor.set(lMotor.getPosition());
            rMotor.setProfile(kBaseLockControlSlot);
            rMotor.changeControlMode(CANTalon.TalonControlMode.Position);
            rMotor.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            rMotor.set(rMotor.getPosition());
            driveControlState = DriveControlState.BASE_LOCKED;
            setBrakeMode(true);
        }
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }


    public double getLeftDistanceInches() {
    	//TODO: figure out why getPosition() and getSpeed() return negative values
        return -rotationsToInches(lMotor.getPosition() / Constants.kQuadEncoderTicksPerRev);
    }

    public double getRightDistanceInches() {
        return -rotationsToInches(rMotor.getPosition() / Constants.kQuadEncoderTicksPerRev);
    }

    public double getLeftVelocityInchesPerSec() {
    	double ticksPer100ms = lMotor.getSpeed();
    	double revPerSec = ticksPer100ms / Constants.kQuadEncoderTicksPerRev * 10;
    	return -rotationsToInches(revPerSec);
    }

    public double getRightVelocityInchesPerSec() {
    	double ticksPer100ms = rMotor.getSpeed();
    	double revPerSec = ticksPer100ms / Constants.kQuadEncoderTicksPerRev * 10;
    	return -rotationsToInches(revPerSec);
    }

    public synchronized void resetEncoders() {
        lMotor.setPosition(0);
        rMotor.setPosition(0);

        lMotor.setEncPosition(0);
        rMotor.setEncPosition(0);
    }

    public synchronized DriveControlState getControlState() {
        return driveControlState;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() 
    {
    	SmartDashboard.putNumber("lMotorCurrent",   lMotor.getOutputCurrent());
    	SmartDashboard.putNumber("rMotorCurrent",   rMotor.getOutputCurrent());
    	SmartDashboard.putNumber("lMotorCtrl",      lMotor.get());
    	SmartDashboard.putNumber("rMotorCtrl",  	rMotor.get());
		SmartDashboard.putNumber("lDistance", getLeftDistanceInches());
		SmartDashboard.putNumber("rDistance", getRightDistanceInches());
        SmartDashboard.putNumber("lVelocity", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("rVelocity", getRightVelocityInchesPerSec());
		//TODO: add closed loop error
        SmartDashboard.putNumber("gyroAngle", imu.getHeading());
		//TODO: add heading error
        
        imu.updateDashboard(9);        
    }

    @Override
    public synchronized void zeroSensors() {
        resetEncoders();
        //imu.reset();
    }

    private void configureTalonsForSpeedControl() {
        if (driveControlState != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState != DriveControlState.VELOCITY_SETPOINT
                && driveControlState != DriveControlState.PATH_FOLLOWING_CONTROL) {
            lMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
            lMotor.setProfile(kVelocityControlSlot);
            lMotor.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rMotor.changeControlMode(CANTalon.TalonControlMode.Speed);
            rMotor.setProfile(kVelocityControlSlot);
            rMotor.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
        }
    }

    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState == DriveControlState.VELOCITY_HEADING_CONTROL
                || driveControlState == DriveControlState.VELOCITY_SETPOINT
                || driveControlState == DriveControlState.PATH_FOLLOWING_CONTROL) {
            lMotor.set(inchesPerSecondToRpm(left_inches_per_sec));
            rMotor.set(inchesPerSecondToRpm(right_inches_per_sec));
        } else {
            System.out.println("Hit a bad velocity control state");
            lMotor.set(0);
            rMotor.set(0);
        }
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public void setBrakeMode(boolean setting) {
        if (brakeMode != setting) {
            lMotor.enableBrakeMode(setting);
            rMotor.enableBrakeMode(setting);
            brakeMode = setting;
        }
    }
    
    public BNO055 getImu() {
    	return imu;
    }
    
}
