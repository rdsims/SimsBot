package org.usfirst.frc.team686.simsbot;

// Adapted from FRC Team 3620, The Average Joes
// https://github.com/FRC3620/FRC3620_2015_AverageJava/blob/master/FRC3620_2015_AverageJava/src/org/usfirst/frc3620/DataLogger.java

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DataLogger 
{
    private static DataLogger mInstance = new DataLogger();
    private static DataLogger mAutonomousInstance = new DataLogger();
    private static DataLogger mVisionInstance = new DataLogger();

    public static DataLogger getInstance() {
        return mInstance;
    }

    public static DataLogger getAutonomousInstance() {
        return mAutonomousInstance;
    }
    
    public static DataLogger getVisionInstance() {
        return mVisionInstance;
    }
    
	static File parentDirectory;
	String fileBase;
	long minimumInterval = 0;	// 0: write as fast as possible

	public enum OutputMode {
		SMARTDASHBOARD_ONLY, FILE_ONLY, SMARTDASHBOARD_AND_FILE
	}
	
	private static OutputMode mOutputMode = OutputMode.SMARTDASHBOARD_ONLY;
	
	static public void findLogDirectory()
	{

		// Determine folder for log files
		File logDirectory = null;
		if (logDirectory == null) 
			logDirectory = checkLogDirectory(new File("/media/sda1"));
		if (logDirectory == null)
		{
			logDirectory = new File("/home/lvuser/logs");
		    if (!logDirectory.exists())
		    {
			    logDirectory.mkdir();
		    }
		}
		if (logDirectory != null && logDirectory.isDirectory())
		{
			String logMessage = String.format("Log directory is %s\n", logDirectory);
			System.out.print (logMessage);
			setDirectory(logDirectory);
			//setMinimumInterval(1000);
		}	        
	}
	
	static public File checkLogDirectory (File root)
	{
		// does the root directory exist?
		if (!root.isDirectory()) 
			return null;
		
		File logDirectory = new File(root, "logs");
		if (!logDirectory.isDirectory()) 
			return null;
		
		return logDirectory;
	}
	
	
	
	static public void setDirectory(File directory) 
	{
		parentDirectory = directory;
	}
	
	public void setFileBase(String _fileBase) 
	{
		fileBase = _fileBase;
	}
	
	List<String> dataNames = new ArrayList<>();
	List<String> dataValues = new ArrayList<>();
	
	public void putBoolean(String name, boolean value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);

		SmartDashboard.putBoolean(name, value);
	}
	
	
	public static void setOutputMode(OutputMode loc) {
		mOutputMode = loc;
	}
	
	public void putNumber(String name, double value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);

		SmartDashboard.putNumber(name, value);
	}
	
	public void putNumber(String name, int value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
		
		SmartDashboard.putNumber(name, value);
	}
	
	public void putString(String name, String value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
		
		SmartDashboard.putString(name, value);
	}
	
	PrintStream ps;
	long startTime;
	long timeUpdated;
	long timeSinceLog;
	
	public boolean shouldLogData() {
		long now = System.currentTimeMillis();
		if((ps==null) || (now - timeSinceLog) > minimumInterval)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	
	public void saveDataItems()
	{
		if(shouldLogData())
		{
			try
			{
				if (ps == null)
				{
					synchronized (this)
					{
						if (ps == null) 
						{
							String timestampString = LogTimestamp.getTimestampString() + "_" + fileBase ;
							if (timestampString != null) 
							{
								String filename = timestampString + "_" + fileBase + ".csv";
								File logFile = new File(parentDirectory, filename);
								ps = new PrintStream(new FileOutputStream(logFile));
								ps.print("time,timeSinceStart");
								writeList(ps, dataNames);
								startTime = System.currentTimeMillis();
							}
						}
					}
				}
				if (ps != null)
				{
					if ((mOutputMode==OutputMode.FILE_ONLY) || (mOutputMode==OutputMode.SMARTDASHBOARD_AND_FILE))
					{
						timeUpdated = (System.currentTimeMillis()-startTime);
						ps.print(getDate());
						ps.print(',');
						ps.print(timeUpdated);
						writeList(ps, dataValues);
						ps.flush();
						timeSinceLog = System.currentTimeMillis();
					}
				}
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
		
		dataValues.clear();
		dataNames.clear();
		
	}
	
	private void writeList(PrintStream stream, List<String> list)
	{
		for(int i = 0;i < list.size(); i++)
		{
			stream.print(',');
			stream.print(list.get(i));
		}
		stream.println();
	}
	
	public String getDate()
	{
		Date curDate = new Date();
		String DateToStr = format.format(curDate);
		return DateToStr;
	}
	
	SimpleDateFormat format = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss.SS");
	
	public void setMinimumInterval(long minimumInterval) 
	{
		this.minimumInterval = minimumInterval;
	}
}
