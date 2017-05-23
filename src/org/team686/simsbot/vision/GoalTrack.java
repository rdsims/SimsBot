package org.team686.simsbot.vision;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import org.team686.lib.util.Vector2d;
import org.team686.simsbot.Constants;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class that is used to keep track of all goals detected by the vision
 * system. As goals are detected/not detected anymore by the vision system,
 * function calls will be made to create, destroy, or update a goal track.
 * 
 * This helps in the goal ranking process that determines which goal to fire
 * into, and helps to smooth measurements of the goal's location over time.
 * 
 * @see GoalTracker.java
 */
public class GoalTrack
{
	Map<Double, Vector2d> mObservedPositions = new TreeMap<>();
	Vector2d mSmoothedPosition = null;
	int mId;

	private GoalTrack()
	{
	}

	/**
	 * Makes a new track based on the timestamp and the goal's coordinates (from
	 * vision)
	 */
	public static GoalTrack makeNewTrack(double timestamp, Vector2d firstObservation, int id)
	{
		GoalTrack track = new GoalTrack();
		track.mObservedPositions.put(timestamp, firstObservation);
		track.mSmoothedPosition = firstObservation;
		track.mId = id;
		return track;
	}

	public void emptyUpdate()
	{
		pruneByTime();
	}

	/**
	 * Attempts to update the track with a new observation.
	 * 
	 * @return True if the track was updated
	 */
	public boolean tryUpdate(double timestamp, Vector2d newObservation)
	{
		if (!isAlive())
		{
			return false;
		}
		double distance = mSmoothedPosition.distance(newObservation);
		if (distance < Constants.kMaxTrackerDistance)
		{
			mObservedPositions.put(timestamp, newObservation);
			pruneByTime();
			return true;
		} 
		else
		{
			// new observation was too far from smoothed observation -- ignore it
			emptyUpdate();
			return false;
		}
	}

	public boolean isAlive()
	{
		return mObservedPositions.size() > 0;
	}

	/**
	 * Removes the track if it is older than the set "age" described in the
	 * Constants file.
	 * 
	 * @see Constants.java
	 */
	void pruneByTime()
	{
		double deleteBefore = Timer.getFPGATimestamp() - Constants.kMaxGoalTrackAge;
		for (Iterator<Map.Entry<Double, Vector2d>> it = mObservedPositions.entrySet().iterator(); it.hasNext();)
		{
			Map.Entry<Double, Vector2d> entry = it.next();
			if (entry.getKey() < deleteBefore)
			{
				it.remove();
			}
		}
		if (mObservedPositions.isEmpty())
		{
			mSmoothedPosition = null;
		} 
		else
		{
			smooth();
		}
	}

	/**
	 * Averages out the observed positions based on an set of observed positions
	 */
	void smooth()
	{
		if (isAlive())
		{
			double x = 0;
			double y = 0;
			for (Map.Entry<Double, Vector2d> entry : mObservedPositions.entrySet())
			{
				x += entry.getValue().getX();
				y += entry.getValue().getY();
			}
			x /= mObservedPositions.size();
			y /= mObservedPositions.size();
			mSmoothedPosition = new Vector2d(x, y);
		}
	}

	public Vector2d getSmoothedPosition()
	{
		return mSmoothedPosition;
	}

	public double getLatestTimestamp()
	{
		return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
	}

	public double getStability()
	{
		return Math.min(1.0, mObservedPositions.size() / (Constants.kCameraFrameRate * Constants.kMaxGoalTrackAge));
	}

	public int getId()
	{
		return mId;
	}
}
