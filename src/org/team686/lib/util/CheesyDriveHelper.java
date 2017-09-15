package org.team686.lib.util;

import org.team686.simsbot.command_status.DriveCommand;
import org.team686.simsbot.subsystems.Drive;

/**
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class CheesyDriveHelper
{
	private static CheesyDriveHelper instance = new CheesyDriveHelper();
	public static CheesyDriveHelper getInstance() { return instance; }
	
	private static final double kThrottleDeadband = 0.02;
	private static final double kWheelDeadband = 0.02;

	// These factor determine how fast the wheel traverses the "non linear" sine curve.
	private static final double kHighWheelNonLinearity = 0.65;
	private static final double kLowWheelNonLinearity = 0.5;

	private static final double kHighNegInertiaScalar = 4.0;

	private static final double kLowNegInertiaThreshold = 0.65;
	private static final double kLowNegInertiaTurnScalar = 3.5;
	private static final double kLowNegInertiaCloseScalar = 4.0;
	private static final double kLowNegInertiaFarScalar = 5.0;

	private static final double kHighSensitivity = 0.95;
	private static final double kLowSensitiity = 1.3;

	private static final double kQuickStopDeadband = 0.2;
	private static final double kQuickStopWeight = 0.1;
	private static final double kQuickStopScalar = 5.0;

	private static double mOldWheel = 0.0;
	private static double mQuickStopAccumlator = 0.0;
	private static double mNegInertiaAccumlator = 0.0;

	public static DriveCommand cheesyDrive(DriveCommand cmd, double throttle, double wheel, boolean isQuickTurn) 
	{
		
		wheel = handleDeadband(wheel, kWheelDeadband);
		throttle = handleDeadband(throttle, kThrottleDeadband);

		// TODO: automatically switch on isQuickTurn for low throttle
		//isQuickTurn |=  (Math.abs(throttle) < kThrottleDeadband);
		
		double negInertia = wheel - mOldWheel;
		mOldWheel = wheel;

		double wheelNonLinearity;
		boolean isHighGear = cmd.getHighGear();
		
		if (isHighGear)
		{
			wheelNonLinearity = kHighWheelNonLinearity;
			final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			// Apply a sin function that's scaled to make it feel better.
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		}
		else
		{
			wheelNonLinearity = kLowWheelNonLinearity;
			final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			// Apply a sin function that's scaled to make it feel better.
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
		}

		double sensitivity;

		// Negative inertia!
		double negInertiaScalar;
		if (isHighGear)
		{
			negInertiaScalar = kHighNegInertiaScalar;
			sensitivity = kHighSensitivity;
		}
		else
		{
			if (wheel * negInertia > 0)
			{
				// If we are moving away from 0.0, aka, trying to get more wheel
				negInertiaScalar = kLowNegInertiaTurnScalar;
			}
			else
			{
				// Otherwise, we are attempting to go back to 0.0.
				if (Math.abs(wheel) > kLowNegInertiaThreshold)
				{
					negInertiaScalar = kLowNegInertiaFarScalar;
				}
				else
				{
					negInertiaScalar = kLowNegInertiaCloseScalar;
				}
			}
			sensitivity = kLowSensitiity;
		}
		double negInertiaPower = negInertia * negInertiaScalar;
		mNegInertiaAccumlator += negInertiaPower;

		wheel = wheel + mNegInertiaAccumlator;
		if (mNegInertiaAccumlator > 1)
		{
			mNegInertiaAccumlator -= 1;
		}
		else if (mNegInertiaAccumlator < -1)
		{
			mNegInertiaAccumlator += 1;
		}
		else
		{
			mNegInertiaAccumlator = 0;
		}

		double linearPower = throttle;

		double overPower, angularPower;
		
		// Quickturn!
		if (isQuickTurn)
		{
			if (Math.abs(linearPower) < kQuickStopDeadband)
			{
				double alpha = kQuickStopWeight;
				mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
										   + alpha * Util.limit(wheel, 1.0) * kQuickStopScalar;
			}
			overPower = 1.0;
			angularPower = wheel;
		}
		else
		{
			overPower = 0.0;
			angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
			if (mQuickStopAccumlator > 1)
			{
				mQuickStopAccumlator -= 1;
			}
			else if (mQuickStopAccumlator < -1)
			{
				mQuickStopAccumlator += 1;
			}
			else
			{
				mQuickStopAccumlator = 0.0;
			}
		}

		double lMotorSpeed = linearPower + angularPower;
		double rMotorSpeed = linearPower - angularPower;

		if (lMotorSpeed > 1.0)
		{
			rMotorSpeed -= overPower * (lMotorSpeed - 1.0);
			lMotorSpeed = 1.0;
		}
		else if (rMotorSpeed > 1.0)
		{
			lMotorSpeed -= overPower * (rMotorSpeed - 1.0);
			rMotorSpeed = 1.0;
		}
		else if (lMotorSpeed < -1.0)
		{
			rMotorSpeed += overPower * (-1.0 - lMotorSpeed);
			lMotorSpeed = -1.0;
		}
		else if (rMotorSpeed < -1.0)
		{
			lMotorSpeed += overPower * (-1.0 - rMotorSpeed);
			rMotorSpeed = -1.0;
		}
		
		cmd.setMotors(lMotorSpeed, rMotorSpeed);	// update the l/r motor speeds in current drive command
		
		return cmd;
	}

	
	public static double handleDeadband(double val, double deadband) 
	{
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
}
