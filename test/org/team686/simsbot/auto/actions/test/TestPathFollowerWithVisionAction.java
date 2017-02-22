package org.team686.simsbot.auto.actions.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.team686.lib.util.Path;
import org.team686.lib.util.PathSegment;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;
import org.team686.lib.util.Path.Waypoint;
import org.team686.simsbot.Constants;
import org.team686.simsbot.auto.actions.PathFollowerWithVisionAction;
import org.team686.simsbot.command_status.RobotState;


public class TestPathFollowerWithVisionAction
{
	Pose actualRobotLocation;
	Pose actualTargetLocation;

	PathFollowerWithVisionAction pathVisionDriveAction;
	RobotState robotState = RobotState.getInstance();
	double currentTime;
	final double dt = 1.0/50.0;

	final double targetWidth = 10.25;

	ArrayList<Double> cameraTimestampQueue;
	ArrayList<Double> cameraTargetXQueue;
	ArrayList<Double> cameraTargetWidthQueue;
	
	@BeforeClass
	public static void setUpBeforeClass() throws Exception {}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {}

	@Before
	public void setUp() throws Exception
	{
		actualRobotLocation  = new Pose( 0, 0,  0);
		actualTargetLocation = new Pose( 0,120,  0);

    	PathSegment.PathSegmentOptions path_options   = new PathSegment.PathSegmentOptions(Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel, Constants.kPathFollowingLookahead, false);
    	PathSegment.PathSegmentOptions vision_options = new PathSegment.PathSegmentOptions(Constants.kVisionMaxVel,        Constants.kVisionMaxAccel,        Constants.kPathFollowingLookahead, true);
    	
        List<Waypoint> path = new ArrayList<>();
        path.add(new Waypoint(new Vector2d( 0, 0), path_options));
        path.add(new Waypoint(new Vector2d(96, 0), path_options));
        path.add(new Waypoint(new Vector2d(96,96), vision_options));
        path.add(new Waypoint(new Vector2d( 0,96), vision_options));
		
		pathVisionDriveAction = new PathFollowerWithVisionAction(new Path(path, false));
		currentTime = (double)(System.currentTimeMillis())/1000.0;

		cameraTimestampQueue = new ArrayList<Double>();
		cameraTargetXQueue = new ArrayList<Double>();
		cameraTargetWidthQueue = new ArrayList<Double>();
		
		
		// fill delay queue with initial values
		for (double t=0; t<Constants.kCameraLatencySeconds; t+=dt)
		{
			cameraTimestampQueue.add(-999.0);
			cameraTargetXQueue.add(-999.0);
			cameraTargetWidthQueue.add(-999.0);
		}
		
		pathVisionDriveAction.start(); 

	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void simulateVisionDrive() 
	{
		for (currentTime = 0; currentTime<10; currentTime+=dt)
		{
			if (pathVisionDriveAction.isFinished())
				break;
			
			
			// update RobotPose -- assume robot has moved with speed & curvature for time dt
			double speed     = pathVisionDriveAction.getSpeed();
			double curvature = pathVisionDriveAction.getCurvature();
			
			double dTheta  = speed*dt*curvature;
			double theta_m = actualRobotLocation.getHeadingRad() + dTheta/2;
	
			double D = speed*dt;					// arc-length
			double L = D;							// chord-length (= arc-length if no curvature)
			if (dTheta > 1e-9)
				L = 2*D*Math.sin(dTheta/2)/dTheta;	// adjust chord-length for curvature	
					
			actualRobotLocation = actualRobotLocation.add(Vector2d.magnitudeAngle(L, theta_m));
			actualRobotLocation = actualRobotLocation.turnRad(dTheta);
			
System.out.println("Robot: " + actualRobotLocation + ", AvgTarget: " + pathVisionDriveAction.avgTargetLocation);
			
			// log RobotPose
			Pose observation = actualRobotLocation;
			robotState.addFieldToVehicleObservation(currentTime, observation);

			// calculate relative position of target
			Vector2d robotToTarget = actualTargetLocation.sub(actualRobotLocation);
			double  distToTarget = robotToTarget.length(); 
			double angleToTarget = robotToTarget.angle() - actualRobotLocation.getHeadingRad();
			angleToTarget = Vector2d.normalizeAngle(angleToTarget);	// modulo 2pi
			
			// calculate Vision output
			double imageTimestamp = currentTime;
			double normalizedTargetX = -999.0; 
			double normalizedTargetWidth = -999.0;
			
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
			
System.out.printf("Vision X = % 7.3f, Width = % 7.3f ---- ", normalizedTargetX, normalizedTargetWidth);
			
			
			//---------------------------------------------------
			// Run test
			//---------------------------------------------------
			
			Pose currentPose = robotState.getLatestFieldToVehicle();		
	
			imageTimestamp -= Constants.kCameraLatencySeconds;		// remove camera latency
			
			// calculate target location based on *previous* robot pose
			Pose previousPose = robotState.getFieldToVehicle(imageTimestamp);
	
			//---------------------------------------------------
			// Process
			//---------------------------------------------------
			pathVisionDriveAction.pathVisionDrive(currentTime, currentPose, previousPose, imageTimestamp, normalizedTargetX, normalizedTargetWidth);
	
			//---------------------------------------------------
			// Output: Send drive control
			//---------------------------------------------------
			//drive.driveCurve(speed, curvature, maxSpeed);
			
			// speed & curvature used to update robot position at top of loop
			
			// TODO: output to CSV file
		}		
		
		
		/*****************************************************
		 * test that visionDrive finished in the time allotted
		 ****************************************************/
        assertTrue(pathVisionDriveAction.isFinished());

		/************************************************
		 * test that visionDrive ended up close to target
		 * and pointed at target
		 ***********************************************/

		// calculate relative position of target
		Vector2d robotToTarget = new Vector2d(actualTargetLocation.getPosition()).sub(actualRobotLocation.getPosition());
		double  distToTarget = robotToTarget.length(); 
		double angleToTarget = robotToTarget.angle() - actualRobotLocation.getHeadingRad();
		angleToTarget = Vector2d.normalizeAngle(angleToTarget);	// modulo 2pi
        
		assertEquals(Constants.kPegTargetDistanceThresholdInches, distToTarget, 1);
		assertEquals(angleToTarget, 0, 0.1);
	}

}
