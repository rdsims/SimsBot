package org.team686.simsbot.auto.actions;

import java.util.ArrayList;
import java.util.List;

import org.team686.simsbot.DataLogger;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action 
{

    private Action mCurAction;
    private final ArrayList<Action> mRemainingActions;

    public SeriesAction(List<Action> actions) 
    {
        mRemainingActions = new ArrayList<>(actions.size());
        for (Action action : actions) 
        {
            mRemainingActions.add(action);
        }
        mCurAction = null;
    }

    @Override
    public boolean isFinished() 
    {
        return mRemainingActions.isEmpty() && mCurAction == null;
    }

    @Override
    public void start() 
    {
    }

    @Override
    public void update() 
    {
        if (mCurAction == null) 
        {
            if (mRemainingActions.isEmpty()) 
            {
                return;
            }
            mCurAction = mRemainingActions.remove(0);
            mCurAction.start();
        }
        mCurAction.update();
        if (mCurAction.isFinished()) 
        {
            mCurAction.done();
            mCurAction = null;
        }
    }

    @Override
    public void done() 
    {
    }
    
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (mCurAction == null) 
            	mCurAction.getLogger().log();
	    }
    };
	
    public DataLogger getLogger() { return logger; }
    
}
