package org.team686.lib.joystick;

import org.team686.lib.joystick.JoystickControlsBase;
import org.team686.lib.util.CheesyDriveHelper;
import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.subsystems.Drive;

/**
 * Implements Team 254's Cheesy Drive. "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class CheesyArcadeDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase instance = new CheesyArcadeDriveJoystick();
	public static JoystickControlsBase getInstance() { return instance; }

	private static Drive drive = Drive.getInstance();

    public DriveCommand getDriveCommand()
    {
        DriveCommand cmd = drive.getCommand();	// find out if we are currently in high gear or not
    	double throttle = -mStick.getY();		// TODO: figure out why Y-axis is negated
        double turn     = +mStick.getX();
        boolean isQuickTurn = mStick.getRawButton(Constants.kQuickTurnButton);
        
        return CheesyDriveHelper.cheesyDrive(cmd, throttle, turn, isQuickTurn);
    }

    
    public double handleDeadband(double val, double deadband) 
    {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

}


