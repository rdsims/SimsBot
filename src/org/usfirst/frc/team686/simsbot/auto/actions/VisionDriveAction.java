package org.usfirst.frc.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team686.lib.util.Path;
import org.usfirst.frc.team686.lib.util.RigidTransform2d;
import org.usfirst.frc.team686.lib.util.Rotation2d;
import org.usfirst.frc.team686.lib.util.Translation2d;
import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.simsbot.subsystems.Drive.DriveControlState;

public class VisionDriveAction implements Action {
	private NetworkTable table;
	private RobotState robotState = RobotState.getInstance();
	private Drive drive = Drive.getInstance();
	private double velocity = 0;
	private double distanceToTargetInches;
	private double headingToTargetRadians;
	private double targetLocationX;
	private double targetLocationY;
	private int    filterCnt;
	private double filteredTargetLocationX;
	private double filteredTargetLocationY;
	private boolean done = false;

	public VisionDriveAction(double _velocity) {
		velocity = _velocity;
		table = NetworkTable.getTable("SmartDashboard");
	}

	@Override
	public void start() {
		// setup code, if any
		filterCnt = 0;
	}

	@Override
	public void update() 
	{
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		double imageTimestamp    	 = table.getNumber("imageTimestamp",    0);
		double normalizedTargetX 	 = table.getNumber("targetCenterX",  -999);
		double normalizedTargetWidth = table.getNumber("targetWidth",    -999);

		if (imageTimestamp > 0) 
		{
			// we have valid values from vision co-processor
			
			imageTimestamp -= Constants.kCameraLatencySeconds;		// remove camera latency
			
			// calculate target location based on *previous* robot pose
			RigidTransform2d previousPose = robotState.getFieldToVehicle(imageTimestamp);

// DEBUG: use current pose until we... 			
// TODO: get timestamp synchronization working
previousPose = robotState.getLatestFieldToVehicle();			

			distanceToTargetInches = Constants.kTargetWidthInches / (2.0*normalizedTargetWidth*Constants.kTangentCameraHalfFOV);
			headingToTargetRadians = previousPose.getRotation().getRadians() + (normalizedTargetX*Constants.kCameraHalfFOVRadians);
			targetLocationX = previousPose.getTranslation().getX() + distanceToTargetInches * Math.cos(headingToTargetRadians);
			targetLocationY = previousPose.getTranslation().getY() + distanceToTargetInches * Math.sin(headingToTargetRadians);
			
			// filter target location with exponential averaging
			if (filterCnt == 0)
			{
				filteredTargetLocationX = targetLocationX;
				filteredTargetLocationY = targetLocationY;
			}
			else
			{
				double a = Constants.kTargetLocationFilterConstant;
				filteredTargetLocationX = (1-a)*filteredTargetLocationX + a*targetLocationX;
				filteredTargetLocationY = (1-a)*filteredTargetLocationY + a*targetLocationY;
			}
			filterCnt++;
			
			// calculate distance and heading to target, based on *current* robot pose
			RigidTransform2d  currentPose = robotState.getLatestFieldToVehicle();			
			double dX = filteredTargetLocationX - currentPose.getTranslation().getX();
			double dY = filteredTargetLocationY - currentPose.getTranslation().getY();
		
			distanceToTargetInches = Math.sqrt(dX*dX + dY*dY);
			headingToTargetRadians = Math.atan2(dY, dX);

			
			// Calculate motor settings to turn towards target  
// TODO!!!!			
			
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
