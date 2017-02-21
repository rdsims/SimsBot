package org.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.team686.lib.util.DataLogger;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.subsystems.Drive;

public class VisionTurnAction implements Action {
	private NetworkTable table;
	private Drive drive = Drive.getInstance();
	private double velocity = 0;
	private double imageTimestamp = 0;
	private double targetCenterX;
	private boolean done = false;

	public VisionTurnAction(double _velocity) {
		velocity = _velocity;
		table = NetworkTable.getTable("SmartDashboard");
	}

	@Override
	public void start() {
		// setup code, if any
	}

	@Override
	public void update()
	{
		double lVel, rVel;
		
		imageTimestamp = table.getNumber("timestamp", -999);
		targetCenterX = table.getNumber("targetCenterX", -999);

		if (imageTimestamp > -999) 
		{
			// we have valid values from vision
			if (targetCenterX > 0)
			{
				lVel = +velocity;
				rVel = -velocity;
			}
			else
			{
				lVel = -velocity;
				rVel = +velocity;
			}

			// debug
			System.out.println("Target X = " + targetCenterX);

			drive.setOpenLoop(new DriveCommand(lVel, rVel));
		} 
		else 
		{
			// debug
			System.out.println("Vision NetworkTables not found");

			// invalid value: not sure if we should stop or coast
			drive.stop();
		}

	}

	@Override
	public boolean isFinished() 
	{
		targetCenterX = table.getNumber("targetCenterX", -999);
		done = (Math.abs(targetCenterX) < 0.05);

		if (done) {
			System.out.println("Target X = " + targetCenterX);
			System.out.println("Finished VisionTurnAction");
		}
		return done;
	}

	@Override
	public void done() {
		// cleanup code, if any
		drive.stop();
	}

	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
    public DataLogger getLogger() { return logger; }
	
}
