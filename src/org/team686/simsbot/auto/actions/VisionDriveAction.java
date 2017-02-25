package org.team686.simsbot.auto.actions;

import org.team686.lib.util.DataLogger;
import org.team686.lib.util.Path;
import org.team686.lib.util.PathFollowerWithVisionDriveController;
import org.team686.lib.util.PathFollowerWithVisionDriveController.PathVisionState;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class VisionDriveAction implements Action 
{
	PathFollowerWithVisionDriveController driveCtrl;

    public VisionDriveAction() 
    {
    	// no defined path
    	driveCtrl = new PathFollowerWithVisionDriveController(new Path(), PathVisionState.VISION);	// start in vision state
    }

    public PathFollowerWithVisionDriveController getDriveController() { return driveCtrl; }
    
    @Override
    public void start() 
    {
		System.out.println("Starting VisionDriveAction");
		driveCtrl.start();
    }


    @Override
    public void update() 
    {
    	driveCtrl.update();
	}	
	
	
    @Override
    public boolean isFinished() 
    {
    	return driveCtrl.isFinished();
    }

    @Override
    public void done() 
    {
		System.out.println("Finished VisionDriveAction");
		// cleanup code, if any
		driveCtrl.done();
    }

 
    
    
    public DataLogger getLogger() { return driveCtrl.getLogger(); }
}
