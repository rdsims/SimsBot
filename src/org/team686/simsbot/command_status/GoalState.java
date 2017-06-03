package org.team686.simsbot.command_status;

import java.util.ArrayList;
import java.util.List;


// Sorted list of range & bearing to each tracked target
// First item in list has the highest score (determined by GoalTracker.TrackReportComparator)

public class GoalState
{
	private static GoalState instance = new GoalState();	
	public static GoalState getInstance()
	{
		return instance;
	}

	List<GoalRangeBearing> goalList = new ArrayList<>();	

	synchronized public void clear()					{	goalList.clear();	}
	synchronized public void add(double distanceToGoal, double bearingToGoal, int trackId)	
	{	
		goalList.add(new GoalRangeBearing(distanceToGoal, bearingToGoal, trackId));	
	}

	synchronized public double getBestTargetRange()		{	return goalList.get(0).getRange();		}
	synchronized public double getBestTargetBearing()	{	return goalList.get(0).getBearing();	}
	synchronized public int getBestTargetTrackId()		{	return goalList.get(0).getTrackId();	}


	
	
	 // A container class to specify the range and angle of a goal
	 // with respect to a robot's position and heading. angle. 
	 // It also contains the computer vision's track's ID.
	
	public class GoalRangeBearing 
	{
	    double range;	// in inches
	    double bearing;	// in radians, relative to robot's heading
	    int trackId;

	    public GoalRangeBearing(double _range, double _bearing, int _trackId) 
	    {
	        range = _range;
	        bearing = _bearing;
	        trackId = _trackId;
	    }

	    public double getRange() { return range; }
	    public double getBearing() { return bearing; }
	    public int getTrackId() { return trackId; }
	}	
}
