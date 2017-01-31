package org.usfirst.frc.team686.simsbot.auto.actions;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc.team686.lib.util.Path;
import org.usfirst.frc.team686.lib.util.Pose;
import org.usfirst.frc.team686.lib.util.RigidTransform2d;
import org.usfirst.frc.team686.simsbot.Constants;
import org.usfirst.frc.team686.simsbot.RobotState;
import org.usfirst.frc.team686.simsbot.subsystems.Drive;
import org.usfirst.frc.team686.simsbot.subsystems.Drive.DriveControlState;

import com.team254.frc2016.Kinematics;

public class VisionDriveAction implements Action {
	private NetworkTable table;
	private RobotState robotState = RobotState.getInstance();
	private Drive drive = Drive.getInstance();
	private double speed = 0;
	private double distanceToTargetInches;
	private double headingToTargetRadians;
	private Pose targetLocation;
	private Pose filteredTargetLocation;
	private int  filterCnt;
	private boolean done = false;

	private static final double kLookaheadDist = 2.0;
	private static final double kEpsilon = 1e-9;
	
	public VisionDriveAction(double _speed) {
		speed = _speed;
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
			RigidTransform2d pPose = robotState.getFieldToVehicle(imageTimestamp);	// using CheesyPoof's RigidTransform2d for now
			Pose previousPose = new Pose(pPose.getTranslation().getX(), pPose.getTranslation().getY(), pPose.getRotation().getRadians());
			
// DEBUG: use current pose until we... 			
// TODO: get timestamp synchronization working
pPose = robotState.getLatestFieldToVehicle();	// using CheesyPoof's RigidTransform2d for now			
previousPose = new Pose(pPose.getTranslation().getX(), pPose.getTranslation().getY(), pPose.getRotation().getRadians());

			distanceToTargetInches = Constants.kTargetWidthInches / (2.0*normalizedTargetWidth*Constants.kTangentCameraHalfFOV);
			headingToTargetRadians = previousPose.getTheta() + (normalizedTargetX*Constants.kCameraHalfFOVRadians);
			Pose toTarget = Pose.fromDistanceHeadingRadians(distanceToTargetInches, headingToTargetRadians);
			targetLocation = previousPose.add(toTarget); 
			
			// filter target location with exponential averaging
			if (filterCnt == 0)
				filteredTargetLocation = targetLocation;
			else
				filteredTargetLocation.filterPosition(targetLocation, Constants.kTargetLocationFilterConstant);
			filterCnt++;
			
			// calculate path from *current* robot pose to target
			RigidTransform2d  cPose = robotState.getLatestFieldToVehicle();		// using CheesyPoof's RigidTransform2d for now		
			Pose currentPose = new Pose(cPose.getTranslation().getX(), cPose.getTranslation().getY(), cPose.getRotation().getRadians());

			// Calculate motor settings to turn towards target (following AdaptivePurePursuitController.java)  
			Pose robotToTarget = filteredTargetLocation.sub(currentPose);
			double dTheta = robotToTarget.getTheta() - currentPose.getTheta();								// change in heading from current pose to target (tangent to circle to be travelled)
			double lookaheadDist = Math.min(kLookaheadDist, currentPose.distance(filteredTargetLocation));	// length of chord
			double curvature = 2.0 * Math.sin(dTheta) / lookaheadDist;										// curvature = 1/radius of circle (negative: turn left, positive: turn right)
		
// TODO: add speed control -- acceleration 
			
			drive.driveCurve(speed, curvature, speed);
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
