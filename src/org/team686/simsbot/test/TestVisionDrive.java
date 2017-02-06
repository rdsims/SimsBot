package org.team686.simsbot.test;

import static org.junit.Assert.*;

import java.util.Queue;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.mini2Dx.gdx.math.Vector2;
import org.team686.lib.util.Pose;
import org.team686.lib.util.RigidTransform2d;
import org.team686.lib.util.Rotation2d;
import org.team686.lib.util.Translation2d;
import org.team686.lib.util.Util;
import org.team686.simsbot.Constants;
import org.team686.simsbot.RobotState;
import org.team686.simsbot.auto.actions.VisionDriveAction;


public class TestVisionDrive
{
	Pose actualRobotLocation;
	Pose actualTargetLocation;

	VisionDriveAction visionDriveAction;
	RobotState robotState = RobotState.getInstance();
	double currentTime;
	final double dt = 1.0/50.0;

	final double targetWidth = 10.25;
//	Vector2 targetLeft;
//	Vector2 targetRight;

	Queue<Double> cameraTimestampQueue;
	Queue<Double> cameraTargetXQueue;
	Queue<Double> cameraTargetWidthQueue;
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception 
	{
	}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {
	}

	@Before
	public void setUp() throws Exception
	{
		actualRobotLocation  = new Pose( 0, 0,  0);
		actualTargetLocation = new Pose(96,36,180);

//		Vector2 targetHalfWidth = Util.fromMagnitudeAngleRad(targetWidth/2, actualTargetLocation.getHeadingRad()+Math.PI/2);
//		targetLeft  = new Vector2(actualTargetLocation.getPosition()).add(targetHalfWidth);
//		targetRight = new Vector2(actualTargetLocation.getPosition()).sub(targetHalfWidth);
		
		visionDriveAction = new VisionDriveAction(Constants.kVisionMaxVel, Constants.kVisionMaxAccel);
		currentTime = (double)(System.currentTimeMillis())/1000.0;

		// fill delay queue with initial values
		for (double t=0; t<Constants.kCameraLatencySeconds; t+=dt)
		{
			cameraTimestampQueue.add(-999.0);
			cameraTargetXQueue.add(-999.0);
			cameraTargetWidthQueue.add(-999.0);
		}
		
		visionDriveAction.start(); 

	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void simulateVisionDrive() 
	{
		for (currentTime = 0; currentTime<10; currentTime+=dt)
		{
			if (visionDriveAction.isFinished())
				break;
			
			
			// update RobotPose -- assume robot has moved with speed & curvature for time dt
			double speed     = visionDriveAction.getSpeed();
			double curvature = visionDriveAction.getCurvature();
			
			double dTheta  = speed*dt*curvature;
			double theta_m = actualRobotLocation.getHeadingRad() + dTheta/2;
	
			double D = speed*dt;					// arc-length
			double L = D;							// chord-length
			if (dTheta > 1e-9)
				L = 2*D*Math.sin(dTheta/2)/dTheta;		
					
			actualRobotLocation.add(Util.fromMagnitudeAngleRad(L, theta_m));
			actualRobotLocation.addHeadingRad(dTheta);
			
			// log RobotPose
			RigidTransform2d observation = new RigidTransform2d(new Translation2d(actualRobotLocation.getX(), actualRobotLocation.getY()), Rotation2d.fromRadians(actualRobotLocation.getHeadingRad()));
			robotState.addFieldToVehicleObservation(currentTime, observation);
			
			// calculate relative position of target
			Vector2 robotToTarget = new Vector2(actualTargetLocation.getPosition()).sub(actualRobotLocation.getPosition());
			double  distToTarget = robotToTarget.len(); 
			double angleToTarget = robotToTarget.angleRad();
			
			// calculate Vision output
			double imageTimestamp = currentTime;
			double normalizedTargetX = -999.0; 
			double normalizedTargetWidth = -999.0;
			
			if (Math.abs(angleToTarget) < Constants.kCameraHalfFOVRadians)
			{
				// target is within camera's field of view
				double fovWidth = 2*distToTarget*Math.cos(Constants.kCameraHalfFOVRadians);		// width of camera's field of view at distance D 
				normalizedTargetWidth = targetWidth / fovWidth;
				normalizedTargetX = angleToTarget / Constants.kCameraHalfFOVRadians;
			}
			
			// delay Vision output
			cameraTimestampQueue.add(imageTimestamp);
			cameraTargetXQueue.add(normalizedTargetX);
			cameraTargetWidthQueue.add(normalizedTargetWidth);
	
			imageTimestamp = cameraTimestampQueue.remove();
			normalizedTargetX = cameraTargetXQueue.remove();
			normalizedTargetWidth = cameraTargetWidthQueue.remove();
			
			
			//---------------------------------------------------
			// Run test
			//---------------------------------------------------
			
			RigidTransform2d  cPose = robotState.getLatestFieldToVehicle();		// using CheesyPoof's RigidTransform2d for now.  TODO: replace		
			Pose currentPose = new Pose(cPose.getTranslation().getX(), cPose.getTranslation().getY(), cPose.getRotation().getRadians());
	
			imageTimestamp -= Constants.kCameraLatencySeconds;		// remove camera latency
			
			// calculate target location based on *previous* robot pose
			RigidTransform2d pPose = robotState.getFieldToVehicle(imageTimestamp);	// using CheesyPoof's RigidTransform2d for now  TODO: replace
			Pose previousPose = new Pose(pPose.getTranslation().getX(), pPose.getTranslation().getY(), pPose.getRotation().getRadians());
	
			//---------------------------------------------------
			// Process
			//---------------------------------------------------
			visionDriveAction.visionDrive(imageTimestamp, normalizedTargetX, normalizedTargetWidth, currentPose, previousPose);
	
			//---------------------------------------------------
			// Output: Send drive control
			//---------------------------------------------------
			//drive.driveCurve(speed, curvature, maxSpeed);
			
			// speed & curvature used to update robot position at top of loop
			
			// TODO: output to CSV file
		}		
		
		//TODO: assert robot is close to target
	}

}
