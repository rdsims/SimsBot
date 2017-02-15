package org.team686.simsbot;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Adapted from FRC Team 3620, The Average Joes
//https://github.com/FRC3620/FRC3620_2015_AverageJava/blob/master/FRC3620_2015_AverageJava/src/org/usfirst/frc3620/DataLogger.java

public class DataLogController
{
	// define separate static instances for each thread
	public static DataLogController robotInstance = new DataLogController("robot");			// log controller for the main robot thread 
	public static DataLogController getRobotLogController() { return robotInstance; } 

	public static DataLogController autoInstance = new DataLogController("auto");			// log controller for the autonomous thread
	public static DataLogController getAutoLogController() { return autoInstance; } 
	
	public DataLogController(String _base)
	{
		findLogDirectory();
		setFileBase(_base);
	}
	
	static File parentDirectory;
	String fileBase;
	long minimumInterval = 0;	// 0: write as fast as possible
	
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
	

	public boolean fileOutput = false;
	public boolean sdOutput = false;
	
    public void setOutputMode(boolean _file, boolean _sd)
    { 
    	fileOutput = _file;
    	sdOutput   = _sd;
    }
    
    
    
    
	
    private final List<DataLogger> loggers  = new ArrayList<>();
	
    public synchronized void register(DataLogger logger) 
    {
        loggers.add(logger);
    }
	
    public synchronized void deregister() 
    {
    	clearLogs();
        loggers.clear();
    }
	
	public void log() 
	{
        for (DataLogger logger : loggers) 
        {
            logger.log();	// collect values to log
        }
		saveDataItems();	// write to file / SmartDashboard
	}

	
	
	
	
	PrintStream ps;
	long startTime;
	long timeUpdated;
	long timeSinceLog;
	
	private boolean shouldLogData() 
	{
		boolean retVal = false;
		
		if (ps==null)
			retVal = true;

		long now = System.currentTimeMillis();
		if ((now - timeSinceLog) > minimumInterval)
			retVal = true;
			
		return retVal;
	}
	
	
	private void saveDataItems()
	{
		if (shouldLogData())
		{
			try
			{
				if (fileOutput && (ps == null))
				{
					// file stream has not yet been initialized
					synchronized (this)
					{
						String timestampString = LogTimestamp.getTimestampString();
						if (timestampString != null) 
						{
							String filename = timestampString + "_" + fileBase + ".csv";
							File logFile = new File(parentDirectory, filename);
							ps = new PrintStream(new FileOutputStream(logFile));
							ps.print("time,timeSinceStart");
							writeNames();
							startTime = System.currentTimeMillis();
						}
					}
				}
				else
				{
					if (fileOutput)
					{
						timeUpdated = (System.currentTimeMillis()-startTime);
						ps.print(getDate());
						ps.print(',');
						ps.print(timeUpdated);
						writeValues();
						ps.flush();
						timeSinceLog = System.currentTimeMillis();
					}
					if (sdOutput)
					{
						putValues();
					}
				}
			}
			catch(IOException e)
			{
				e.printStackTrace();
			}
		}
		
		clearLogs();
		
	}

	private void writeNames() 
	{
        for (DataLogger logger : loggers) 
        {
    		for (String name : logger.logMap.keySet())
    		{
    			ps.print(',');
    			ps.print(name);
    		}
        }
		ps.println();
	}
	
	private void writeValues() 
	{
        for (DataLogger logger : loggers) 
        {
        	for (Object value : logger.logMap.values())
        	{
    			ps.print(',');
    			ps.print(value.toString());
        	}
        }
		ps.println();
	}
	
	private void putValues() 
	{
        for (DataLogger logger : loggers) 
        {
        	for (Map.Entry<String, Object> entry : logger.logMap.entrySet())
        	{
        		String key = entry.getKey();
        		Object value = entry.getValue();
        		
        		if (value.getClass().equals(Boolean.class))
        			putValue(key, (Boolean)value);
        		else if (value.getClass().equals(Integer.class))
        			putValue(key, (Integer)value);
        		else if (value.getClass().equals(Double.class))
        			putValue(key, (Double)value);        		
        		else if (value.getClass().equals(Float.class))
        			putValue(key, (Double)value);        		
        		else if (value.getClass().equals(String.class))
        			putValue(key, (String)value);
        		else
        			putValue(key, "ERROR");
        	}
        }
	}
	
	private void putValue(String key, boolean val) { SmartDashboard.putBoolean(key, val); }
	private void putValue(String key, int val)     { SmartDashboard.putNumber(key, val); }
	private void putValue(String key, double val)  { SmartDashboard.putNumber(key, val); }
	private void putValue(String key, String val)  { SmartDashboard.putString(key, val); }
	
	private void clearLogs() 
	{
        for (DataLogger logger : loggers) 
        {
    		logger.logMap.clear();
        }
	}
	

        
	private String getDate()
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
