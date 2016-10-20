package org.usfirst.frc.team686.simsbot.subsystems;

//import java.util.Set;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import org.usfirst.frc.team686.lib.sensors.BNO055;
import org.usfirst.frc.team686.lib.util.AdaptivePurePursuitController;
import org.usfirst.frc.team686.lib.util.DriveSignal;
import org.usfirst.frc.team686.lib.util.Path;
import org.usfirst.frc.team686.lib.util.RigidTransform2d;
import org.usfirst.frc.team686.lib.util.Rotation2d;
import org.usfirst.frc.team686.lib.util.SynchronousPID;
import org.usfirst.frc.team686.lib.util.Util;

import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.DataLogger;
import org.usfirst.frc.team686.simsbot.Kinematics;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.loops.Loop;


/**
 * The robot's drivetrain, which implements the Superstructure abstract class.
 * The drivetrain has several states and builds on the abstract class by
 * offering additional control methods, including control by path and velocity.
 * 
 * @see Subsystem.java
 */
public class Drive extends Subsystem 
{
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static Drive instance_ = new Drive();

    public static Drive getInstance() {
        return instance_;
    }

    private double mLastHeadingErrorDegrees;
    
    // The robot drivetrain's various states
    public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL, PATH_FOLLOWING_CONTROL
    }

    public final CANTalon lMotor_, rMotor_;
    private boolean isBrakeMode_ = true;

    private DriveControlState driveControlState_;
    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
    private AdaptivePurePursuitController pathFollowingController_;
    private SynchronousPID velocityHeadingPid_;
    
    private static BNO055 imu = BNO055.getInstance(Constants.BNO055_PORT);

    // The main control loop (an implementation of Loop), which cycles
    // through different robot states
    private final Loop mLoop = new Loop() 
    {
        @Override
        public void onStart() 
        {
            setOpenLoop(DriveSignal.NEUTRAL);
            pathFollowingController_ = null;
            setBrakeMode(false);
//            stopOnNextCount_ = false;
        }

        @Override
        public void onLoop() 
        {
            synchronized (Drive.this) 
            {
/*            	
                if (stopOnNextCount_ && getSeesLineCount() > lastSeesLineCount_) 
                {
                    poseWhenStoppedOnLine_ = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                    stopOnNextCount_ = false;
                    stop();
            	}
*/
                switch (driveControlState_) 
                {
                case OPEN_LOOP:
                    return;
                case BASE_LOCKED:
                    return;
                case VELOCITY_SETPOINT:
                    // Talons are updating the control loop state
                    return;
                case VELOCITY_HEADING_CONTROL:
                    updateVelocityHeadingSetpoint();
                    return;
                case PATH_FOLLOWING_CONTROL:
                    updatePathFollower();
                    if (isFinishedPath()) {
                        stop();
                    }
                    break;
                default:
                    System.out.println("Unexpected drive control state: " + driveControlState_);
                    break;
                }
            }
        }

        @Override
        public void onStop() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };

    
    
    
    // The constructor instantiates all of the drivetrain components when the
    // robot powers up
    private Drive() 
    {
        lMotor_ = new CANTalon(Constants.kLeftMotorTalonId);
        rMotor_ = new CANTalon(Constants.kRightMotorTalonId);

        // Get status at 100Hz
        lMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rMotor_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

        // Start in open loop mode
        lMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        lMotor_.set(0);
        rMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rMotor_.set(0);
        setBrakeMode(false);

        // Set up the encoders
        lMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (lMotor_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }
        lMotor_.setInverted(false);
        lMotor_.reverseSensor(true);
        lMotor_.reverseOutput(false);
        rMotor_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rMotor_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }
        lMotor_.setInverted(true);		// right motor is flipped with respect to left motor.  Need to reverse direction of rotation in PercentVbus mode
        rMotor_.reverseSensor(false);	// inverts feedback in closed loop modes
        rMotor_.reverseOutput(true);	// reverse direction of rotation in closed loop modes

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



        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi,
                Constants.kDriveHeadingVelocityKd);
        velocityHeadingPid_.setOutputRange(-30, 30);

        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public Loop getLoop() {
        return mLoop;
    }

    
    public DriveSignal tankDrive(double throttle, double turn)
    {
	    boolean squaredInputs = true;
	    
	    double moveValue   = Util.limit(throttle, 1.0);
	    double rotateValue = Util.limit(turn,     1.0);
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
        lMotor_.set(left);
        rMotor_.set(right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            lMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rMotor_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.lMotor, signal.rMotor);
    }

    public synchronized void setBaseLockOn() {
        if (driveControlState_ != DriveControlState.BASE_LOCKED) {
            lMotor_.setProfile(kBaseLockControlSlot);
            lMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
            lMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            lMotor_.set(lMotor_.getPosition());
            rMotor_.setProfile(kBaseLockControlSlot);
            rMotor_.changeControlMode(CANTalon.TalonControlMode.Position);
            rMotor_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            rMotor_.set(rMotor_.getPosition());
            driveControlState_ = DriveControlState.BASE_LOCKED;
            setBrakeMode(true);
        }
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    /**
     * The robot follows a set path, which is defined by Waypoint objects.
     * 
     * @param Path
     *            to follow
     * @param reversed
     * @see com.team254.lib.util/Path.java
     */
    public synchronized void followPath(Path path, boolean reversed) {
        if (driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.PATH_FOLLOWING_CONTROL;
            velocityHeadingPid_.reset();
        }
        pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
                Constants.kPathFollowingMaxAccel, Constants.kLoopDt, path, reversed, 0.25);
        updatePathFollower();
    }

    /**
     * @return Returns if the robot mode is Path Following Control and the set
     *         path is complete.
     */
    public synchronized boolean isFinishedPath() {
        return (driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController_.isDone())
                || driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL;
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(lMotor_.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rMotor_.getPosition());
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(lMotor_.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(rMotor_.getSpeed());
    }

    public synchronized void resetEncoders() {
        lMotor_.setPosition(0);
        rMotor_.setPosition(0);

        lMotor_.setEncPosition(0);
        rMotor_.setEncPosition(0);
    }

    public synchronized DriveControlState getControlState() {
        return driveControlState_;
    }

    public BNO055 getImu() {
        return imu;
    }
    
    public double getHeading() {
        return imu.getHeading();
    }
    
    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void log() {
    	DataLogger dataLogger = DataLogger.getInstance();
    	
  		dataLogger.putNumber("lMotorCurrent",  	lMotor_.getOutputCurrent());
		dataLogger.putNumber("rMotorCurrent",  	rMotor_.getOutputCurrent());
		dataLogger.putNumber("lMotorCtrl",     	lMotor_.get());
		dataLogger.putNumber("rMotorCtrl",		rMotor_.get());
		dataLogger.putNumber("lDistance",      	getLeftDistanceInches());
		dataLogger.putNumber("rDistance",      	getRightDistanceInches());
		dataLogger.putNumber("lVelocity", 	 	getLeftVelocityInchesPerSec());
		dataLogger.putNumber("rVelocity", 	 	getRightVelocityInchesPerSec());
		//TODO: add closed loop error
		dataLogger.putNumber("Heading", 	 	getImu().getHeading());
		//TODO: add heading error
    }
/*    
        SmartDashboard.putNumber("left_distance", getLeftDistanceInches());
*/

    @Override
    public synchronized void zeroSensors() {
        resetEncoders();
    }

    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT
                && driveControlState_ != DriveControlState.PATH_FOLLOWING_CONTROL) {
            lMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
            lMotor_.setProfile(kVelocityControlSlot);
            lMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rMotor_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rMotor_.setProfile(kVelocityControlSlot);
            rMotor_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
        }
    }

    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL
                || driveControlState_ == DriveControlState.VELOCITY_SETPOINT
                || driveControlState_ == DriveControlState.PATH_FOLLOWING_CONTROL) {
            lMotor_.set(inchesPerSecondToRpm(left_inches_per_sec));
            rMotor_.set(inchesPerSecondToRpm(right_inches_per_sec));
        } else {
            System.out.println("Hit a bad velocity control state");
            lMotor_.set(0);
            rMotor_.set(0);
        }
    }

    private void updateVelocityHeadingSetpoint() {
        Rotation2d actualGyroAngle = Rotation2d.fromDegrees(getHeading());

        mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse())
                .getDegrees();

        double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
        updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2,
                velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
    }

    private void updatePathFollower() {
        RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
        Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.left));
        max_vel = Math.max(max_vel, Math.abs(setpoint.right));
        if (max_vel > Constants.kPathFollowingMaxVel) {
            double scaling = Constants.kPathFollowingMaxVel / max_vel;
            setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
        }
        updateVelocitySetpoint(setpoint.left, setpoint.right);
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
            lMotor_.enableBrakeMode(on);
            rMotor_.enableBrakeMode(on);
            isBrakeMode_ = on;
        }
    }
    
    
    /**
     * VelocityHeadingSetpoints are used to calculate the robot's path given the
     * speed of the robot in each wheel and the polar coordinates. Especially
     * useful if the robot is negotiating a turn and to forecast the robot's
     * location.
     */
    public static class VelocityHeadingSetpoint {
        private final double leftSpeed_;
        private final double rightSpeed_;
        private final Rotation2d headingSetpoint_;

        // Constructor for straight line motion
        public VelocityHeadingSetpoint(double leftSpeed, double rightSpeed, Rotation2d headingSetpoint) {
            leftSpeed_ = leftSpeed;
            rightSpeed_ = rightSpeed;
            headingSetpoint_ = headingSetpoint;
        }

        public double getLeftSpeed() {
            return leftSpeed_;
        }

        public double getRightSpeed() {
            return rightSpeed_;
        }

        public Rotation2d getHeading() {
            return headingSetpoint_;
        }
    }    
}
