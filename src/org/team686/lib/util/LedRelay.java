package org.team686.lib.util;

import org.team686.simsbot.Constants;

import edu.wpi.first.wpilibj.Relay;

// class that only allows one instance of LED control relay
// needed because wpilibj only allows Relay() to be constructed once
public class LedRelay
{
	public static Relay instance = new Relay(Constants.kLedRelayPort, Relay.Direction.kForward);
	public static Relay getInstance() { return instance; }
}
