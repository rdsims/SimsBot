package org.usfirst.frc.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team686.lib.util.Path;
import org.usfirst.frc.team686.lib.util.Rotation2d;
import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.simsbot.subsystems.Drive.DriveControlState;

public class VisionDriveAction implements Action 
{
	private NetworkTable table;
	private Drive drive = Drive.getInstance();
	private double velocity = 0;
	private double targetCenterX;
	private double targetWidth;
	private double targetHeading = 0; // target heading in radians
	private boolean done = false;

	public VisionDriveAction(double _velocity) {
		velocity = _velocity;
		table = NetworkTable.getTable("SmartDashboard");
	}

	@Override
	public void start() {
		// setup code, if any
		targetHeading = drive.getHeading(); // start with current heading,
											// correct from there
	}

	@Override
	public void update() {
		targetCenterX = table.getNumber("targetCenterX", -999);
		targetWidth    = table.getNumber("targetWidth", -999);

		if (targetWidth > 0) {
			// we have valid values from vision
			if (Math.abs(targetCenterX) > 0.01)
				targetHeading -= Math.signum(targetCenterX) * 1;

			// debug
			System.out.println("Target X = " + targetCenterX + ", W: " + targetWidth + ", Heading = " + targetHeading);

			drive.setVelocityHeadingSetpoint(velocity, Rotation2d.fromDegrees(targetHeading));
		} else {
			// debug
			System.out.println("Vision NetworkTables not found");

			// invalid value: not sure if we should stop or coast
			drive.stop();
		}

	}

	@Override
	public boolean isFinished() {
		done = (targetWidth > Constants.kPegTargetWidthThreshold);
		if (done) {
			System.out.println("Finished VisionDriveAction");
		}
		return done;
	}

	@Override
	public void done() {
		// cleanup code, if any
		drive.stop();
	}

}
