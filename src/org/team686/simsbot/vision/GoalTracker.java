package org.team686.simsbot.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import org.team686.lib.util.Vector2d;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.GoalStates.GoalState;

import edu.wpi.first.wpilibj.Timer;

/**
 * This is used in the event that multiple goals are detected to judge all goals
 * based on timestamp, stability, and continuation of previous goals (i.e. if a
 * goal was detected earlier and has changed locations). This allows the robot
 * to make consistent decisions about which goal to aim at and to smooth out
 * jitter from vibration of the camera.
 * 
 * @see GoalTrack.java
 */
public class GoalTracker
{
	/**
	 * Track reports contain all of the relevant information about a given goal
	 * track.
	 */
	public static class TrackReport
	{
		
		public Vector2d fieldToGoal;		// Translation from the field frame to the goal
		public double latestTimestamp;		// The timestamp of the latest time that the goal has been observed
		public double stability;			// The percentage of the goal tracking time during which this goal has been observed (0 to 1)
		public int trackId;					// The track id

		public TrackReport(GoalTrack track)
		{
			fieldToGoal = track.getSmoothedPosition();
			latestTimestamp = track.getLatestTimestamp();
			stability = track.getStability();
			trackId = track.getId();
		}

		public Vector2d getFieldToGoal() {	return fieldToGoal;	}
		public double getLatestTimestamp()	{	return latestTimestamp;	}
		public double getStability()	{	return stability;	}
		public int getTrackId()	{	return trackId;	}	
	}

	
	
	
	
	/**
	 * TrackReportComparators are used in the case that multiple tracks are
	 * active (e.g. we see or have recently seen multiple goals). They contain
	 * heuristics used to pick which track we should aim at by calculating a
	 * score for each track (highest score wins).
	 */
	public static class TrackReportComparator implements Comparator<TrackReport>
	{
		// Reward tracks for being more stable (seen in more frames)
		double stabilityWeight;
		// Reward tracks for being recently observed
		double ageWeight;
		double currentTimestamp;
		// Reward tracks for being continuations of tracks that we are already
		// tracking
		double switchingWeight;
		int currentBestTrackId;

		public TrackReportComparator(double stability_weight, double age_weight, double switching_weight,
				int current_best_track_id, double current_timestamp)
		{
			stabilityWeight = stability_weight;
			ageWeight = age_weight;
			switchingWeight = switching_weight;
			currentBestTrackId = current_best_track_id;
			currentTimestamp = current_timestamp;
		}

		double score(TrackReport report)
		{
			double stabilityScore = stabilityWeight * report.stability;
			double trackAge = (currentTimestamp - report.latestTimestamp);
			double ageScore = ageWeight	* Math.max(0, 1 - trackAge / Constants.kGoalTrackAveragePeriod);
			double switchingScore = (report.trackId == currentBestTrackId ? switchingWeight : 0);
			return (stabilityScore + ageScore + switchingScore);
		}

		@Override
		public int compare(TrackReport o1, TrackReport o2)
		{
			double diff = score(o1) - score(o2);
			
			// rv>0 if o1 is better than o2
			// rv=0 if o1 is equal to o2
			// rv<0 if o1 is worse than o2
			int rv = 0;
			if (diff < 0)
				rv = 1;
			else if (diff > 0)
				rv = -1;

			return rv;
		}
	}

	List<GoalTrack> currentTracks = new ArrayList<>();
	int currentBestTrackId = -1;
	int mNextId = 0;

	public GoalTracker()
	{
	}

	public void reset()
	{
		currentTracks.clear();
	}

	public void update(double imageTimestamp, List<Vector2d> fieldToGoals)
	{
		boolean hasUpdatedTrack = false;
		
		// Try to update existing tracks
		for (Vector2d target : fieldToGoals)
		{
			for (GoalTrack track : currentTracks)
			{
				if (!hasUpdatedTrack)
				{
					// see if this goal location is close to any previously found goals
					if (track.tryUpdate(imageTimestamp, target))
					{
						hasUpdatedTrack = true;
					}
				}
				else
				{
					track.emptyUpdate();
				}
			}
		}
		
		// Prune any tracks that have died
		for (Iterator<GoalTrack> it = currentTracks.iterator(); it.hasNext();)
		{
			GoalTrack track = it.next();
			if (!track.isAlive())
			{
				it.remove();
			}
		}
		
		// If all tracks are dead, start new tracks for any detections
		if (currentTracks.isEmpty())
		{
			for (Vector2d target : fieldToGoals)
			{
				currentTracks.add(GoalTrack.makeNewTrack(imageTimestamp, target, mNextId));
				++mNextId;
			}
		}
	}


	
	
	public boolean hasTracks()
	{
		return !currentTracks.isEmpty();
	}

	public List<TrackReport> getTrackReports()
	{
        double now = Timer.getFPGATimestamp();
		
		List<TrackReport> rv = new ArrayList<>();
		for (GoalTrack track : currentTracks)
		{
			// only return tracks that have been updated recently
			if (now - track.getLatestTimestamp() <= Constants.kMaxTargetAge)
			{
				rv.add(new TrackReport(track));
			}
		}
		return rv;
	}

	public List<TrackReport> getSortedTrackReports(double currentTime, Optional<GoalState> currentTarget)
	{
		// Sort tracks (actually TrackReports) so we can identify the best track
		if (currentTarget.isPresent())
			currentBestTrackId = currentTarget.get().getTrackId();
		else
			currentBestTrackId = -1;
			
		// define a comparator so that GoalTracks can be sorted by rank
	    GoalTracker.TrackReportComparator goalTrackComparator = new GoalTracker.TrackReportComparator(
	    		Constants.kTrackReportComparatorStablityWeight, Constants.kTrackReportComparatorAgeWeight, 
	    		Constants.kTrackReportComparatorSwitchingWeight, currentBestTrackId, currentTime);
		
		List<GoalTracker.TrackReport> trackReports = getTrackReports();
		Collections.sort(trackReports, goalTrackComparator);	// sort tracks by rank
		
		return trackReports;
	}
}
