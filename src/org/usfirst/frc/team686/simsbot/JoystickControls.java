package org.usfirst.frc.team686.simsbot;

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
        return mStick.getY();	// TODO: figure out why X-axis is throttle
    }

    public double getTurn() {
        return mStick.getX();	// TODO: figure out why Y-axis is turning
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
