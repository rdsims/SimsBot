package org.team686.simsbot.auto.actions.test;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.team686.lib.util.DataLogController;
import org.team686.lib.util.Kinematics;
import org.team686.lib.util.Path;
import org.team686.lib.util.PathFollowerWithVisionDriveController;
import org.team686.lib.util.PathSegment;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;
import org.team686.lib.util.Kinematics.WheelSpeed;
import org.team686.lib.util.MyTimer;
import org.team686.lib.util.Path.Waypoint;
import org.team686.lib.util.PathFollowerWithVisionDriveController.PathVisionState;
import org.team686.simsbot.Constants;
import org.team686.simsbot.auto.*;
import org.team686.simsbot.auto.modes.AutoPlacePegMode;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.command_status.DriveState;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.loops.DriveLoop;
import org.team686.simsbot.loops.GoalStateLoop;
import org.team686.simsbot.loops.LoopController;
import org.team686.simsbot.loops.RobotStateLoop;
import org.team686.simsbot.vision.VisionState;

import edu.wpi.first.wpilibj.Timer;

import org.team686.simsbot.subsystems.Drive;



public class TestAutonomousModes
{
	Pose robotPose;
	Pose targetPose;

	Drive drive = Drive.getInstance();
	DriveState driveStatus = DriveState.getInstance();
	RobotState robotState = RobotState.getInstance();
	VisionState visionStatus = VisionState.getInstance();
	
	PathFollowerWithVisionAction pathVisionDriveAction;
	PathFollowerWithVisionDriveController driveCtrl;

	double currentTime;
	final double dt = 1.0/50.0;

	final double targetWidth = 10.25;

	ArrayList<Double> cameraTimestampQueue;
	ArrayList<Double> cameraTargetXQueue;
	ArrayList<Double> cameraTargetWidthQueue;
	
	DataLogController testLogger;
	
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {}

	@Before
	public void setUp() throws Exception
	{
		testLogger = DataLogController.getRobotLogController();
	}

	@After
	public void tearDown() throws Exception {} 

	
	// copied from Robot.java
	public void setInitialPose(Pose _initialPose)
	{
		robotState.reset(Timer.getFPGATimestamp(), DriveState.getInstance().getLeftDistanceInches(),
				DriveState.getInstance().getRightDistanceInches(), _initialPose);

		System.out.println("InitialPose: " + _initialPose);
	}
	
	
	@Test 
	public void testAutonomous()
	{
		// robotInit()
		TestLoopController loopController = new TestLoopController();
		loopController.register(drive.getVelocityPIDLoop());
		loopController.register(DriveLoop.getInstance());
		loopController.register(RobotStateLoop.getInstance());
		loopController.register(GoalStateLoop.getInstance());
		
		
		testLogger.deregister();
		testLogger.register(drive.getCommand().getLogger());
		testLogger.register(driveStatus.getLogger());
		testLogger.register(robotState.getLogger());
		testLogger.register(visionStatus.getLogger());
		testLogger.setOutputMode(true, false);

		MyTimer.setMode(MyTimer.TimestampMode.SIMULATED);
		
		
		// autonomousPeriodic()
		AutoModeExecuter autoModeExecuter = new AutoModeExecuter();
		autoModeExecuter.setAutoMode( new AutoPlacePegMode(1, true) );
		
		setInitialPose(autoModeExecuter.getAutoMode().getInitialPose());

		
				
		// autonomousPeriodic()
		currentTime = 0;
		while (currentTime <= 15)	// autonomous period lasts 15 seconds
		{
			// get robot update every kLoopDt
			autoModeExecuter.getAutoMode().run();
			loopController.run();

			// simulate robot physics at a higher timing resolution
			simulateRobotPhysics(Constants.kLoopDt);
			currentTime += Constants.kLoopDt;
			
			
			// test stuff executed every timestep
			errorCheckEachTimestep();
			// printout

			
			// get ready for next timestep
			MyTimer.update(Constants.kLoopDt);
		}
		
		errorCheckAtEnd();		
	}
	

	public void simulateRobotPhysics(double _dt)
	{
    	// simulate physics of mechanism over the robot's time step
    	double kSimTime = _dt/100.0;	// simulate physics at a higher time resolution
    	double t = 0.0;
    	
    	while (t < _dt)
    	{
    		t += kSimTime;
    	}		
	}
	
	
	public void errorCheckEachTimestep()
	{
		 // test that did not stray from path
		if (driveCtrl.getPathVisionState() == PathVisionState.PATH_FOLLOWING)
		{
			double distFromPath = driveCtrl.getDistanceFromPath(); 
			assertTrue(distFromPath < 12);
		}
	}
	
	public void errorCheckAtEnd()
	{
		 // test that we finished in the time allotted
        assertTrue(pathVisionDriveAction.isFinished());


        // if final segment has vision enabled, check that we ended near the target
        if (driveCtrl.getPath().getSegmentVisionEnable())
        {
			// calculate relative position of target
			Vector2d robotToTarget = new Vector2d(targetPose.getPosition()).sub(robotPose.getPosition());
			double  distToTarget = robotToTarget.length(); 
			double angleToTarget = robotToTarget.angle() - robotPose.getHeading();
			angleToTarget = Vector2d.normalizeAngle(angleToTarget);	// modulo 2pi
	        
	        // test that visionDrive ended up close to target and pointed at target
			assertEquals(Constants.kPegTargetDistanceThresholdFromCameraInches, distToTarget, 1);
			assertEquals(angleToTarget, 0, 0.1);
        }
	}
	
	
	
	Pose currentPose;
	Pose previousPose;
	
	double imageTimestamp = currentTime;
	double normalizedTargetX = -999.0; 
	double normalizedTargetWidth = -999.0;
	
	public void simulateTimestep() 
	{
		// update RobotPose -- assume robot has moved with speed & curvature for time dt
		
		simulateDriveLoop();
		simulateRobotStateLoop();
		
		robotPose = robotState.getLatestFieldToVehicle();
		
		// calculate relative position of target
		Vector2d robotToTarget = targetPose.getPosition().sub(robotPose.getPosition());
		double  distToTarget = robotToTarget.length(); 
		double angleToTarget = robotToTarget.angle() - robotPose.getHeading();
		angleToTarget = Vector2d.normalizeAngle(angleToTarget);	// modulo 2pi
		
		// calculate Vision output
		imageTimestamp = currentTime;
		normalizedTargetX = -999.0; 
		normalizedTargetWidth = -999.0;
		
		if (Math.abs(angleToTarget) < Constants.kCameraHalfFOVRadians)
		{
			// target is within camera's field of view
			double fovWidth = 2*distToTarget*Constants.kTangentCameraHalfFOV;		// width of camera's field of view at distance D 
			normalizedTargetWidth = targetWidth / fovWidth;
			normalizedTargetX = -angleToTarget / Constants.kCameraHalfFOVRadians;
		}
		
		// delay Vision output
		cameraTimestampQueue.add(imageTimestamp);
		cameraTargetXQueue.add(normalizedTargetX);
		cameraTargetWidthQueue.add(normalizedTargetWidth);

		imageTimestamp = cameraTimestampQueue.remove(0);
		normalizedTargetX = cameraTargetXQueue.remove(0);
		normalizedTargetWidth = cameraTargetWidthQueue.remove(0);
		
		simulateVisionLoop();
		
		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		simulationUpdate();
		
		//---------------------------------------------------
		// Log
		//---------------------------------------------------
		testLogger.log();
		
		System.out.printf("RobotPose: %s\n", robotPose);
	}

	// equivalent to PathFollowerWithVisionDriveController.update(), but for off-robot testing
    public void simulationUpdate() 
    {
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		imageTimestamp    	  = visionStatus.getImageCaptureTimestamp();
		normalizedTargetX 	  = visionStatus.getNormalizedTargetX();
		normalizedTargetWidth = visionStatus.getNormalizedTargetWidth();

		currentPose  = robotState.getLatestFieldToVehicle();				
		previousPose = robotState.getFieldToVehicle(imageTimestamp);

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		WheelSpeed wheelSpeed = driveCtrl.pathVisionDrive(currentTime, currentPose, previousPose, imageTimestamp, normalizedTargetX, normalizedTargetWidth);	// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control 
		// (won't actually go to motors in simulation)
		//---------------------------------------------------
        drive.setVelocitySetpoint(wheelSpeed);
	}
	
    
	public void simulateDriveLoop()
	{
		DriveCommand newCmd = drive.getCommand();
		
		// copy commands over to status, as if Talon's performed perfectly
		driveStatus.setTalonControlMode( newCmd.getTalonControlMode() );
		driveStatus.setNeutralMode(        newCmd.getNeutralMode() );
		
		// get encoder values from hardware, set in Drive
		double lSpeed = newCmd.getLeftMotor();
		double rSpeed = newCmd.getRightMotor();

		driveStatus.setLeftDistanceInches(  driveStatus.getLeftDistanceInches()  + lSpeed * dt );
		driveStatus.setRightDistanceInches( driveStatus.getRightDistanceInches() + rSpeed * dt );

		driveStatus.setLeftSpeedInchesPerSec(  lSpeed );
		driveStatus.setRightSpeedInchesPerSec( rSpeed );

		Kinematics.LinearAngularSpeed speed = Kinematics.forwardKinematics(lSpeed, rSpeed);

		driveStatus.setHeading( driveStatus.getHeading() + speed.angularSpeed*dt );

		if (newCmd.getResetEncoders())
		{
			driveStatus.setLeftDistanceInches(  0 );
			driveStatus.setRightDistanceInches( 0 );

			driveStatus.setLeftSpeedInchesPerSec(  0 );
			driveStatus.setRightSpeedInchesPerSec( 0 );
			
			// can't reset gyro, depend on RobotState.gyroCorrection
		}
	}

	
	public void simulateRobotStateLoop()
	{
		double time		 = currentTime;
        double lDistance = driveStatus.getLeftDistanceInches();
        double rDistance = driveStatus.getRightDistanceInches();
        double lSpeed    = driveStatus.getLeftSpeedInchesPerSec();
        double rSpeed    = driveStatus.getRightSpeedInchesPerSec(); 
        double gyroAngle = driveStatus.getHeading();

        robotState.generateOdometryFromSensors(time, lDistance, rDistance, lSpeed, rSpeed, gyroAngle);
	}
    
	
	public void simulateVisionLoop()
	{
		visionStatus.setImageTimestamp( 		imageTimestamp );
		visionStatus.setNormalizedTargetX( 		normalizedTargetX );
		visionStatus.setNormalizedTargetWidth( 	normalizedTargetWidth );
	}	
	
}
