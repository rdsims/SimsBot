package org.team686.simsbot.vision;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.team686.lib.util.DataLogger;


import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * VisionState contains the various attributes calculated by the vision system,
 * namely a list of targets and the timestamp at which it was captured.
 */
public class VisionState
{
	private static VisionState instance = new VisionState();
	public static VisionState getInstance()
	{
		return instance;
	}
	
	protected List<VisionTargetState> targets = new ArrayList<>();
	protected double imageCaptureTimestamp = 0;			// note: assumes transport time from phone to this code is instantaneous

	private static JSONParser parser = new JSONParser();
    private ArrayList<VisionStateListener> listeners = new ArrayList<>();

	
	
	/**
	 * Generates a VisionState object given a JSON blob and a timestamp.
	 * 
	 * @param Capture
	 *            timestamp
	 * @param JSON
	 *            blob with update string, example: 
	 *            { "capturedAgoMs" : 100,
	 *              "targets": [{"hAngle": 0.2, "vAngle": 0.4, "hWidth": 0.1, "vWidth": 0.1}] }
	 * @return VisionState object
	 */
	public void updateFromJsonString(double currentTime, String updateString)
	{
		try
		{
			JSONObject j = (JSONObject) parser.parse(updateString);
			Optional<Long> capturedAgoMs = parseLong(j, "capturedAgoMs");
			if (!capturedAgoMs.isPresent())
			{
				// invalid TargetState
				return;
			}
			imageCaptureTimestamp = currentTime - (capturedAgoMs.get() / 1000.0);		// assumes transport time from phone to this code is instantaneous
			
			JSONArray jTargets = (JSONArray) j.get("targets");
			ArrayList<VisionTargetState> targetStates = new ArrayList<>(jTargets.size());
			for (Object jTargetObj : jTargets)
			{
				JSONObject jTarget = (JSONObject) jTargetObj;
				Optional<Double> hAngle = parseDouble(jTarget, "hAngle");
				Optional<Double> vAngle = parseDouble(jTarget, "vAngle");
				Optional<Double> hWidth = parseDouble(jTarget, "hWidth");
				Optional<Double> vWidth = parseDouble(jTarget, "vWidth");
				if (!(hAngle.isPresent() && vAngle.isPresent() && hWidth.isPresent() && vWidth.isPresent()))
				{
					// invalid TargetState
					return;
				}
				targetStates.add(new VisionTargetState(hAngle.get(), vAngle.get(), hWidth.get(), vWidth.get()));
			}
			
			// if all target info in JSON was valid, make local copy
			setTargets( targetStates );
			
			// notify all listeners that we have an updated VisionState
			for (VisionStateListener listener : listeners)
			{
				listener.visionStateNotify();
			}
		}
		catch (ParseException e)
		{
			System.err.println("Parse error: " + e);
			System.err.println(updateString);
		}
		catch (ClassCastException e)
		{
			System.err.println("Data type error: " + e);
			System.err.println(updateString);
		}
	}
	
	
	
	private static Optional<Long> parseLong(JSONObject j, String key) throws ClassCastException
	{
		Object val = j.get(key);
		if (val == null)
		{
			return Optional.empty();
		}
		return Optional.of((long) val);
	}


	private static Optional<Double> parseDouble(JSONObject j, String key) throws ClassCastException
	{
		Object val = j.get(key);
		if (val == null)
		{
			return Optional.empty();
		}
		return Optional.of((double) val);
	}

	
	
	public void addVisionStateListener(VisionStateListener listener) 
	{
	     if (!listeners.contains(listener))
	     {
	     	listeners.add(listener);
	     }
	}
	
	public void removeVisionStateListener(VisionStateListener listener) 
	{
	    if (listeners.contains(listener)) 
	    	listeners.remove(listener);
	}
	
    
	// Synchronized get/set functions for access from other threads

	synchronized public void setTargets(List<VisionTargetState> _targets) 	{ targets = _targets; }
	
	synchronized public List<VisionTargetState> getTargets() 	{ return targets; }

	synchronized public double getImageCaptureTimestamp()	{ return imageCaptureTimestamp; }


	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
			int k=0;
			for (VisionTargetState target : targets)
			{
				put(String.format("VisionState/Target%d/hCenter", k), target.hCenter);
				put(String.format("VisionState/Target%d/hWidth",  k), target.hWidth);
				put(String.format("VisionState/Target%d/vCenter", k), target.vCenter);
				put(String.format("VisionState/Target%d/vWidth",  k), target.vWidth);
				k++;
			}
		}
	};

	public DataLogger getLogger()
	{
		return logger;
	}

}
