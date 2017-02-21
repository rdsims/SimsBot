package org.team686.simsbot.command_status;

public class VisionStatus
{
	private static VisionStatus instance = new VisionStatus();
	public static VisionStatus getInstance() { return instance; }

	private double imageTimestamp;

	private double normalizedTargetX;
	private double normalizedTargetWidth;
	
	public synchronized void   setImageTimestamp( double _val ) { imageTimestamp = _val; }
	public synchronized double getImageTimestamp() { return imageTimestamp; }
	
	public synchronized void   setNormalizedTargetX( double _val ) { normalizedTargetX = _val; }
	public synchronized double getNormalizedTargetX() { return normalizedTargetX; }

	public synchronized void   setNormalizedTargetWidth( double _val ) { normalizedTargetWidth = _val; }
	public synchronized double getNormalizedTargetWidth() { return normalizedTargetWidth; }
}
