package org.team686.simsbot.loops;

import org.team686.simsbot.command_status.RobotState;
import org.team686.simsbot.vision.VisionUpdate;
import org.team686.simsbot.vision.VisionUpdateReceiver;

/**
 * This function adds vision updates (from the Nexus smartphone) to a list in
 * RobotState. This helps keep track of goals detected by the vision system. The
 * code to determine the best goal to shoot at and prune old Goal tracks is in
 * GoalTracker.java
 * 
 * @see GoalTracker.java
 */
public class VisionProcessor implements Loop, VisionUpdateReceiver
{
	static VisionProcessor instance = new VisionProcessor();
	VisionUpdate nextUpdate = null;
	RobotState robotState = RobotState.getInstance();

	public static VisionProcessor getInstance()
	{
		return instance;
	}

	VisionProcessor()
	{
	}

	@Override
	public void onStart()
	{
	}

	@Override
	public void onLoop()
	{
		VisionUpdate thisUpdate;
		synchronized (this)
		{
			if (nextUpdate == null)
			{
				return;
			}
			thisUpdate = nextUpdate;
			nextUpdate = null;
		}
		robotState.addVisionTargets(thisUpdate.getCapturedAtTimestamp(), thisUpdate.getTargets());
	}

	@Override
	public void onStop()
	{
		// no-op
	}

	@Override
	public synchronized void gotUpdate(VisionUpdate _update)
	{
		nextUpdate = _update;
	}

}
