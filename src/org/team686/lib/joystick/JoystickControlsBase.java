package org.team686.lib.joystick;

import edu.wpi.first.wpilibj.Joystick;
import org.team686.lib.util.DriveCommand;

/**
 * An abstract class that forms the base of joystick controls.
 */
public abstract class JoystickControlsBase 
{
    protected final Joystick mStick;

    protected JoystickControlsBase() 
    {
        mStick = new Joystick(0);
    }

    // DRIVER CONTROLS
    public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveSignal
}
