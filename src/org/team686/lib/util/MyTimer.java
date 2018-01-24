package org.team686.lib.util;

import edu.wpi.first.wpilibj.Timer;

// Hardware Abstraction Layer class
// can be configured to return
// 1) FPGA timestamp (normal case)
// 2) Java timestamp (for non-robot testing)
// 3) simulated timestamp (time ticks occur when commanded)

public class MyTimer
{
	private static MyTimer instance = new MyTimer();
	public static MyTimer getInstance() { return instance; }
	
	public static enum TimestampMode { FPGA, JAVA, SIMULATED };
	private static TimestampMode timestampMode = TimestampMode.FPGA;
	private static double simulatedTimestamp = 0;

	
	MyTimer()
	{
		timestampMode = TimestampMode.FPGA;	// override with setMode() if different mode is desired
		simulatedTimestamp = 0;
	}
	
	public static void setMode(TimestampMode _timestampMode)
	{
		timestampMode = _timestampMode;
	}
	
	public static double getTimestamp()
	{
		switch (timestampMode) 
		{
		case FPGA:
		default:
			return Timer.getFPGATimestamp();

		case JAVA:
			return System.currentTimeMillis();

		case SIMULATED:
			return simulatedTimestamp;
		}
	}

	public static void update(double _dt)
	{
		simulatedTimestamp += _dt;
	}
}
