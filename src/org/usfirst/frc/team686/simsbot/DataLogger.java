package org.usfirst.frc.team686.simsbot;

// Adapted from FRC Team 3620, The Average Joes
// https://github.com/FRC3620/FRC3620_2015_AverageJava/blob/master/FRC3620_2015_AverageJava/src/org/usfirst/frc3620/DataLogger.java

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintStream;

// TODO: script file to move logs off of flash drive (not just copy) but only if successfully transferred

public class DataLogger 
{
    private static DataLogger mInstance = new DataLogger();

    public static DataLogger getInstance() {
        return mInstance;
    }

	File parentDirectory;
	long minimumInterval = 0;	// 0: write as fast as possible

	
	public void findLogDirectory()
	{

		// Determine folder for log files
		File logDirectory = null;
		if (logDirectory == null) logDirectory = checkLogDirectory(new File("/media/sda1"));
		if (logDirectory == null)
		{
			logDirectory = new File("/home/admin/logs");
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
	
	public File checkLogDirectory (File root)
	{
		// does the root directory exist?
		if (!root.isDirectory()) return null;
		
		File logDirectory = new File(root, "logs");
		if (!logDirectory.isDirectory()) return null;
		
		return logDirectory;
	}
	
	
	
	public void setDirectory(File directory) 
	{
		parentDirectory = directory;
	}
	
	List<String> dataNames = new ArrayList<>();
	List<String> dataValues = new ArrayList<>();
	public void addDataItem(String name, double value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
	}
	
	public void addDataItem(String name, int value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
	}
	
	public void addDataItem(String name, boolean value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
	}
	
	public void addDataItem(String name, String value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
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
							String timestampString = LogTimestamp.getTimestampString();
							if (timestampString != null) 
							{
								File logFile = new File(parentDirectory, timestampString + ".csv");
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
					timeUpdated = (System.currentTimeMillis()-startTime);
					ps.print(getDate());
					ps.print(',');
					ps.print(timeUpdated);
					writeList(ps, dataValues);
					ps.flush();
					timeSinceLog = System.currentTimeMillis();
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
