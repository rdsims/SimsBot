package org.team686.simsbot.command_status;

import java.util.ArrayList;
import java.util.List;

import org.team686.lib.util.DataLogger;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;


// Sorted list of range & bearing to each tracked target
// First item in list has the highest score (determined by GoalTracker.TrackReportComparator)

public class GoalStates
{
	private static GoalStates instance = new GoalStates();	
	public static GoalStates getInstance()
	{
		return instance;
	}

	List<GoalState> goalList = new ArrayList<>();	

	synchronized public void clear()					{	goalList.clear();	}
	synchronized public void add(Vector2d _fieldToGoal, Pose _fieldToShooter, int _trackId)	
	{	
		goalList.add(new GoalState(_fieldToGoal, _fieldToShooter, _trackId));	
	}

	synchronized public boolean targetFound()					 {	return (goalList.size() > 0);		}
	synchronized public int getNumTargets()						 {	return goalList.size();		}
	synchronized public Vector2d getBestTargetPosition()		 {	return goalList.get(0).getPosition();		}
	synchronized public double getBestTargetHorizontalDistance() {	return goalList.get(0).getHorizontalDistance();		}
	synchronized public double getBestTargetRelativeBearing()	 {	return goalList.get(0).getRelativeBearing();	}
	synchronized public int getBestTargetTrackId()				 {	return goalList.get(0).getTrackId();	}


	
	
	 // A container class to specify the range and angle of a goal
	 // with respect to a robot's position and heading. angle. 
	 // It also contains the computer vision's track's ID.
	
	public class GoalState 
	{
		Vector2d fieldToGoal;
	    double horizontalDistance;	// in inches
	    double relativeBearing;		// in radians, relative to robot's heading
	    int trackId;

	    public GoalState(Vector2d _fieldToGoal, Pose _fieldToShooter, int _trackId) 
	    {
			// find relative distance and bearing to goal
			Vector2d shooterToGoal = _fieldToGoal.sub(_fieldToShooter.getPosition());
	    	
			double distanceToGoal = shooterToGoal.length();
			double bearingToGoal = shooterToGoal.angle() - _fieldToShooter.getHeading(); 	// bearing relative to shooter's heading
		
			fieldToGoal = _fieldToGoal;
	        horizontalDistance = distanceToGoal;
	        relativeBearing = bearingToGoal;
	        trackId = _trackId;
	    }

	    public Vector2d getPosition() { return fieldToGoal; }
	    public double getHorizontalDistance() { return horizontalDistance; }
	    public double getRelativeBearing() { return relativeBearing; }
	    public int getTrackId() { return trackId; }
	}	





	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
			synchronized (GoalStates.this)
			{
				put("GoalState/numTargets", goalList.size());
				put("GoalState/bestTargetX", getBestTargetPosition().getX());
				put("GoalState/bestTargetY", getBestTargetPosition().getY());
				put("GoalState/bestTargetRange", getBestTargetHorizontalDistance());
				put("GoalState/bestTargetBearing", getBestTargetRelativeBearing());
			}
		}
	};

	public DataLogger getLogger()
	{
		return logger;
	}
}
