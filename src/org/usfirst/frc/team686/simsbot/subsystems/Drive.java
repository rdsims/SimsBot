package org.usfirst.frc.team686.simsbot.subsystems;

//import java.util.Set;

import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.CANTalon;
//import edu.wpi.first.wpilibj.Counter;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Solenoid;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem {
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static Drive instance_ = new Drive();

    public static Drive getInstance() {
        return instance_;
    }

    // The robot drivetrain's various states
    public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
    }

    public final CANTalon leftMotor_, rightMotor_;
    private boolean isBrakeMode_ = true;

    private DriveControlState driveControlState_;
    
    // The constructor instantiates all of the drivetrain components when the
    // robot powers up
    private Drive() 
    {
        leftMotor_ = new CANTalon(Constants.kLeftMotorTalonId);
        rightMotor_ = new CANTalon(Constants.kRightMotorTalonId);

        // Get status at 100Hz
        leftMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rightMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

        // Start in open loop mode
        leftMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftMotor_.set(0);
        rightMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightMotor_.set(0);
        setBrakeMode(false);

        // Set up the encoders
        leftMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftMotor_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }
        leftMotor_.setInverted(false);
        leftMotor_.reverseSensor(true);
        leftMotor_.reverseOutput(false);
        rightMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightMotor_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }
        leftMotor_.setInverted(true);		// right motor is flipped with respect to left motor.  Need to reverse direction of rotation in PercentVbus mode
        rightMotor_.reverseSensor(false);	// inverts feedback in closed loop modes
        rightMotor_.reverseOutput(true);	// reverse direction of rotation in closed loop modes

        // Load velocity control gains
        leftMotor_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        rightMotor_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        // Load base lock control gains
        leftMotor_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);
        rightMotor_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);

        setOpenLoop(DriveSignal.NEUTRAL);
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
	    double leftMotorSpeed, rightMotorSpeed;
	    
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
	        leftMotorSpeed = moveValue - rotateValue;
	        rightMotorSpeed = Math.max(moveValue, rotateValue);
	      } else {
	        leftMotorSpeed = Math.max(moveValue, -rotateValue);
	        rightMotorSpeed = moveValue + rotateValue;
	      }
	    } else {
	      if (rotateValue > 0.0) {
	        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
	        rightMotorSpeed = moveValue + rotateValue;
	      } else {
	        leftMotorSpeed = moveValue - rotateValue;
	        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
	      }
	    }
	    
	    DriveSignal signal = new DriveSignal(leftMotorSpeed, rightMotorSpeed);
	   	    
	    return signal;
    }

    
    
    
    protected synchronized void setLeftRightPower(double left, double right) {
        leftMotor_.set(left);
        rightMotor_.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            leftMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rightMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }

    public synchronized void setBaseLockOn() {
        if (driveControlState_ != DriveControlState.BASE_LOCKED) {
            leftMotor_.setProfile(kBaseLockControlSlot);
            leftMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
            leftMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            leftMotor_.set(leftMotor_.getPosition());
            rightMotor_.setProfile(kBaseLockControlSlot);
            rightMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
            rightMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            rightMotor_.set(rightMotor_.getPosition());
            driveControlState_ = DriveControlState.BASE_LOCKED;
            setBrakeMode(true);
        }
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }


    public double getLeftDistanceInches() {
        return rotationsToInches(leftMotor_.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rightMotor_.getPosition());
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(leftMotor_.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(rightMotor_.getSpeed());
    }

    public synchronized void resetEncoders() {
        leftMotor_.setPosition(0);
        rightMotor_.setPosition(0);

        leftMotor_.setEncPosition(0);
        rightMotor_.setEncPosition(0);
    }

    public synchronized DriveControlState getControlState() {
        return driveControlState_;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left_distance", getLeftDistanceInches());
        SmartDashboard.putNumber("right_distance", getRightDistanceInches());
        SmartDashboard.putNumber("left_velocity", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right_velocity", getRightVelocityInchesPerSec());
    }

    @Override
    public synchronized void zeroSensors() {
        resetEncoders();
    }

    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT
                && driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            leftMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMotor_.setProfile(kVelocityControlSlot);
            leftMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rightMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMotor_.setProfile(kVelocityControlSlot);
            rightMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
        }
    }

    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
                || driveControlState_ == DriveControlState.VELOCITY_SETPOINT
                || driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) {
            leftMotor_.set(inchesPerSecondToRpm(left_inches_per_sec));
            rightMotor_.set(inchesPerSecondToRpm(right_inches_per_sec));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMotor_.set(0);
            rightMotor_.set(0);
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

    public void setBrakeMode(boolean on) {
        if (isBrakeMode_ != on) {
            leftMotor_.enableBrakeMode(on);
            rightMotor_.enableBrakeMode(on);
            isBrakeMode_ = on;
        }
    }
}
