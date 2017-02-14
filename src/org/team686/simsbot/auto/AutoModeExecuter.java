package org.team686.simsbot.auto;

import org.team686.lib.util.CrashTrackingRunnable;
import org.team686.simsbot.DataLogController;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecuter 
{
    private AutoModeBase m_auto_mode;
    private Thread m_thread = null;
    DataLogController autoLogger;
    
    public void setAutoMode(AutoModeBase new_auto_mode) 
    {
        m_auto_mode = new_auto_mode;
    }

    public AutoModeBase getAutoMode() 
    {
        return m_auto_mode;
    }

    public void start() 
    {
		autoLogger = DataLogController.getAutoLogController();

        if (m_thread == null) 
        {
            m_thread = new Thread(new CrashTrackingRunnable() 
            {
                @Override
                public void runCrashTracked() 
                {
                    if (m_auto_mode != null) 
                    {
                        m_auto_mode.run(autoLogger);
                    }
                }
            });
            m_thread.start();
        }

    }

    public void stop() 
    {
        if (m_auto_mode != null) 
        {
            m_auto_mode.stop();
        }
        m_thread = null;
    }

}
