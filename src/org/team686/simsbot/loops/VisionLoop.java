package org.team686.simsbot.loops;

import org.team686.simsbot.VisionStatus;
import edu.wpi.first.wpilibj.networktables.NetworkTable;


public class VisionLoop implements Loop
{
	private static VisionLoop instance = new VisionLoop();
	public static VisionLoop getInstance() { return instance; }
	
	private NetworkTable table;
	private VisionStatus visionStatus;
	
	
	private VisionLoop() 
	{
		table = NetworkTable.getTable("SmartDashboard");
		visionStatus = VisionStatus.getInstance();
	}
	
	
	@Override public void onStart()
	{
		// nothing
	}

	@Override public void onLoop()
	{
		//---------------------------------------------------
		// Get values from NetworkTable and write to VisionStatus structure
		//---------------------------------------------------
		
		synchronized (visionStatus)		// lock visionStatus while we update all variables
		{
			// values from camera, normalized to camera's Field of View (-1 to +1) 
			visionStatus.setImageTimestamp( 		table.getNumber("Camera/imageTimestamp",    0) );
			visionStatus.setNormalizedTargetX( 		table.getNumber("Camera/targetCenterX",  -999) );
			visionStatus.setNormalizedTargetWidth( 	table.getNumber("Camera/targetWidth",    -999) );
		}
	}

	@Override public void onStop()
	{
	}

};
