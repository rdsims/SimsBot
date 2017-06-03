package org.team686.simsbot.loops;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.GoalStates;
import org.team686.simsbot.command_status.RobotState;
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

	GoalTracker goalTracker;
	GoalStates goalStates = GoalStates.getInstance();

	int currentBestTrackId = -1;
	
	
	public static GoalStateLoop getInstance()
	{
		return instance;
	}

	GoalStateLoop()
	{
	}

	@Override
	public void onStart()
	{
	}

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
        double differentialHeight = Constants.kCenterOfTargetHeight - Constants.kCameraPoseZ;	
		
		if (!(visionTargets == null || visionTargets.isEmpty()))
		{
			for (VisionTargetState target : visionTargets)
			{
				double hAngle = target.getHorizontalAngle() - Constants.kCameraPoseThetaRad;	// compensate for camera yaw
				double vAngle = target.getVerticalAngle()   - Constants.kCameraPitchRad;		// compensate for camera pitch
				
				// Targets have a known height.  
				// If you know the height and the angle to the target, 
				// you can calculate the horizontal distance to the target
				if (vAngle > 0)
				{
					double distance = differentialHeight / Math.tan(vAngle);
					Pose cameraToTarget = new Pose( Vector2d.magnitudeAngle(distance, hAngle) );
					Pose fieldToTarget = cameraToTarget.changeCoordinateSystem( fieldToCamera );
					
					fieldToGoals.add( fieldToTarget.getPosition() );	
				}
			}
		}
		
	
		
		
		// Step 2: Add these goals to goal tracker
		goalTracker.update(imageCaptureTimestamp, fieldToGoals);
		
		
		
		
		// Step 3: Rank each goal, sort goals by rank
		
		// define a comparator so that GoalTracks can be sorted by rank
        double now = Timer.getFPGATimestamp();
		currentBestTrackId = goalStates.getBestTargetTrackId();
        		
        GoalTracker.TrackReportComparator goalTrackComparator = new GoalTracker.TrackReportComparator(
        		Constants.kTrackReportComparatorStablityWeight, Constants.kTrackReportComparatorAgeWeight, 
        		Constants.kTrackReportComparatorSwitchingWeight, currentBestTrackId, now);
		
		List<GoalTracker.TrackReport> reports = goalTracker.getTracks();
		Collections.sort(reports, goalTrackComparator);	// sort tracks by rank
        
		

		
		
		// Step 4: Store position of goals, calculate range/bearing from shooter to each goal

		Pose predictedFieldToShooter = robotState.getPredictedFieldToShooter(Constants.kAutoAimPredictionTime);

		goalStates.clear();
		for (GoalTracker.TrackReport report : reports)
		{
			goalStates.add(report.fieldToGoal, predictedFieldToShooter, report.trackId);
		}		

	}
	
	
	public synchronized void resetVision()
	{
		goalTracker.reset();
	}

}
