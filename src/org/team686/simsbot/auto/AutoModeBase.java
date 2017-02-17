package org.team686.simsbot.auto;

import org.team686.lib.util.Vector;
import org.team686.simsbot.DataLogController;
import org.team686.simsbot.DataLogger;
import org.team686.simsbot.auto.actions.Action;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto.modes (which are routines that do actions).
 */
public abstract class AutoModeBase
{
    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    static DataLogController autoLogger = DataLogController.getAutoLogController();
    
    public Vector getInitialPosition()
    {
    	return new Vector(0, 0);
    }
    
    protected abstract void routine() throws AutoModeEndedException;

    public void run() 
    {
        m_active = true;
        try 
        {
            routine();
        } 
        catch (AutoModeEndedException e) 
        {
            System.out.println("Auto mode done, ended early");
            return;
        }
        done();
        System.out.println("Auto mode done");
    }

    public void done() 
    {
    }

    public void stop() 
    {
        m_active = false;
    }

    public boolean isActive() 
    {
        return m_active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException 
    {
        if (!isActive()) 
        {
            throw new AutoModeEndedException();
        }
        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException 
    {
        autoLogger.deregister();						// remove previous action loggers from registry
        autoLogger.register(action.getLogger());		// register logger for new action        isActiveWithThrow();
        autoLogger.setOutputMode(true, true);
        
        action.start();
        while (isActiveWithThrow() && !action.isFinished()) 
        {
            action.update();
            autoLogger.log();
            
            long waitTime = (long) (m_update_rate * 1000.0);	// TODO: use timer to get more consistent update periods
            try
            {
                Thread.sleep(waitTime);
            } 
            catch (InterruptedException e) 
            {
                e.printStackTrace();
            }
        }
        action.done();
    }

}
