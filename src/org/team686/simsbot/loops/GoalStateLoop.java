package org.team686.simsbot.loops;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.GoalStates;
import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.command_status.GoalStates.GoalState;
import org.team686.simsbot.vision.GoalTracker;
import org.team686.simsbot.vision.VisionState;
import org.team686.simsbot.vision.VisionStateListener;
import org.team686.simsbot.vision.VisionTargetState;

import edu.wpi.first.wpilibj.Timer;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in
 * RobotState. This helps keep track of goals detected by the vision system. The
 * code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class GoalStateLoop implements Loop, VisionStateListener
{
	static GoalStateLoop instance = new GoalStateLoop();
	boolean visionUpdatePending = false; // set to true when updated vision information is available 
										 // (set from VisionServer thread)

	RobotState robotState = RobotState.getInstance();
	VisionState visionState = VisionState.getInstance();

	GoalTracker goalTracker = new GoalTracker();
	GoalStates goalStates = GoalStates.getInstance();

	int currentBestTrackId = -1;
	
	enum RangeMethod { DIFFERENTIAL_HEIGHT, TARGET_HEIGHT, TARGET_WIDTH };
	final RangeMethod rangeMethod = RangeMethod.TARGET_WIDTH; 
	
	public static GoalStateLoop getInstance()
	{
		return instance;
	}

	GoalStateLoop()
	{
	}

	@Override
	public void onStart() {}

	@Override
	public void onLoop()
	{
		if (visionUpdatePending)
		{
			updateGoalLocations();
			visionUpdatePending = false;
		}
	}

	@Override
	public void onStop()
	{
		// no-op
	}

	@Override
	public void visionStateNotify()
	{
		visionUpdatePending = true;
	}

	private void updateGoalLocations()
	{
		// Step 1: Find location of goals in this image with respect to field
		
		double imageCaptureTimestamp = visionState.getImageCaptureTimestamp();
		List<VisionTargetState> visionTargets = visionState.getTargets();
		Pose fieldToCamera = robotState.getFieldToCamera(imageCaptureTimestamp);	// find position of camera back when image was taken (removes latency in processing)

		List<Vector2d> fieldToGoals = new ArrayList<>();
		
		if (!(visionTargets == null || visionTargets.isEmpty()))
		{
			for (VisionTargetState target : visionTargets)
			{
				double hAngle = target.getHorizontalAngle() - Constants.kCameraPoseThetaRad;	// compensate for camera yaw
				double vAngle = target.getVerticalAngle()   - Constants.kCameraPitchRad;		// compensate for camera pitch
				double hWidth = target.getHorizontalWidth();
				double vWidth = target.getVerticalWidth();
						
		        double differentialHeight = Constants.kCenterOfTargetHeightInches - Constants.kCameraPoseZ;	
				double horizontalDistance = 0;
				double range = 0;
				double cos = 0;
				
				switch (rangeMethod)
				{
				case DIFFERENTIAL_HEIGHT:
					horizontalDistance = Math.abs(differentialHeight / Math.tan(vAngle));
					break;
					
				case TARGET_HEIGHT:
					// assumes target is vertical
					cos = Math.cos(vAngle);
					range = Constants.kTargetHeightInches * cos / vWidth;
					horizontalDistance = range * cos;
					break;
					
				case TARGET_WIDTH:
					// assumes target is horizontally perpendicular to camera axis (not likely unless you attempt to make it so)
					cos = Math.cos(hAngle);
					range = Constants.kTargetWidthInches * cos / hWidth;
					horizontalDistance = range * cos;
					break;
					
				default:
					break;
				}
				
				if (horizontalDistance > 0)
				{
					Pose cameraToTarget = new Pose( Vector2d.magnitudeAngle(horizontalDistance, hAngle) );
					Pose fieldToTarget = cameraToTarget.changeCoordinateSystem( fieldToCamera );
					fieldToGoals.add( fieldToTarget.getPosition() );
				}
			}
		}
		
	
		
		
		// Step 2: Add these goals to goal tracker
		goalTracker.update(imageCaptureTimestamp, fieldToGoals);
		
		

		
		// Step 3: 	Rank each goal, sort goals by rank
		//			Store position of goals, calculate range/bearing from shooter to each goal
        double now = Timer.getFPGATimestamp();
        Optional<GoalState> currentTarget = goalStates.getBestVisionTarget();
		Pose predictedFieldToShooter = robotState.getPredictedFieldToShooter(Constants.kAutoAimPredictionTime);

		goalStates.clear();
		for (GoalTracker.TrackReport report : goalTracker.getSortedTrackReports(now, currentTarget))
		{
			goalStates.add(report.fieldToGoal, predictedFieldToShooter, report.trackId, report.getLatestTimestamp());
		}		

	}
	
	
	public synchronized void resetVision()
	{
		goalTracker.reset();
	}

	
	public GoalTracker getGoalTracker() { return goalTracker; }
	
}
