package org.team686.simsbot;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Adapted from FRC Team 3620, The Average Joes
//https://github.com/FRC3620/FRC3620_2015_AverageJava/blob/master/FRC3620_2015_AverageJava/src/org/usfirst/frc3620/DataLogger.java

public class DataLogController
{
    private static DataLogController mInstance = new DataLogController();
    private static DataLogController mAutonomousInstance = new DataLogController();
    private static DataLogController mVisionInstance = new DataLogController();

    public static DataLogController getInstance() {
        return mInstance;
    }

    public static DataLogController getAutonomousInstance() {
        return mAutonomousInstance;
    }
    
    public static DataLogController getVisionInstance() {
        return mVisionInstance;
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
	

	public enum OutputMode { SMARTDASHBOARD_ONLY, FILE_ONLY, SMARTDASHBOARD_AND_FILE }
	private OutputMode mOutputMode = OutputMode.SMARTDASHBOARD_ONLY;

    public void setOutputMode(OutputMode _mode) { mOutputMode = _mode; }
    
	
    private final List<DataLogger> loggers  = new ArrayList<>();
	
    public synchronized void register(DataLogger logger) 
    {
        loggers.add(logger);
    }
	
	public void log() 
	{
        for (DataLogger logger : loggers) 
        {
            logger.log();
        }
		saveDataItems();
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
ps.print("time,timeSinceStart");
writeList(ps, dataNames);
						timeUpdated = (System.currentTimeMillis()-startTime);
						ps.print(getDate());
						ps.print(',');
						ps.print(timeUpdated);
						writeList(ps, dataValues);
						ps.flush();
						timeSinceLog = System.currentTimeMillis();
					}
					if ((mOutputMode==OutputMode.SMARTDASHBOARD_ONLY) || (mOutputMode==OutputMode.SMARTDASHBOARD_AND_FILE))
					{
						SmartDashboard.putBoolean(name, value);		
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
