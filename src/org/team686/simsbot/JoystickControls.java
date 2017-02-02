package org.team686.simsbot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A basic framework for the control board Like the drive code, one instance of
 * the JoystickReader object is created upon startup, then other methods request
 * the singleton JoystickReader instance.
 */
public class JoystickControls {
    private static JoystickControls mInstance = new JoystickControls();

    public static JoystickControls getInstance() {
        return mInstance;
    }

    private final Joystick mStick;

    private JoystickControls() {
        mStick = new Joystick(0);
    }

    // DRIVER CONTROLS
    public double getThrottle() {
        return mStick.getRawAxis(3) - mStick.getRawAxis(2);
    	//return -mStick.getY();	// TODO: figure out why Y-axis is negated
    }

    public double getTurn() {
    	return mStick.getX();
        //return -mStick.getX();	// TODO: figure out why X-axis is negated
    }

    public boolean getQuickTurn() {
        return mStick.getRawButton(1);
    }

    public boolean getTractionControl() {
        return mStick.getRawButton(2);
    }

    public boolean getLowGear() {
        return mStick.getRawButton(2);
    }

    public boolean getFireButton() {
        return mStick.getRawButton(1);
    }

    // OPERATOR CONTROLS
    public boolean getKeepWheelRunning() {
        return mStick.getRawAxis(3) > 0.1;
    }

}
