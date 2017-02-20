package org.team686.simsbot.auto;

import org.team686.lib.util.Pose;
import org.team686.simsbot.DataLogController;
import org.team686.simsbot.auto.actions.Action;

import edu.wpi.first.wpilibj.Timer;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto.modes (which are routines that do actions).
 */
public abstract class AutoModeBase
{
    protected double m_update_period = 1.0 / 50.0;
    protected boolean m_active = false;

    static DataLogController autoLogger = DataLogController.getAutoLogController();
    
    public Pose getInitialPose()
    {
    	return new Pose();	// default implementation
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
        	double currTime = Timer.getFPGATimestamp();
        	double nextTime = Timer.getFPGATimestamp() + m_update_period;
        	
            action.update();
            autoLogger.log();

        	currTime = Timer.getFPGATimestamp();
            long waitTime = (long) ((nextTime-currTime) * 1000.0);	// attempt to run thread every m_update_period seconds
            waitTime = Math.max(waitTime, 0);						// avoid negative waits
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
        autoLogger.log();	// capture one last log
    }

}
