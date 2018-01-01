package org.team686.lib.sensors;

import org.team686.simsbot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class NavX extends GyroBase
{
	private static GyroBase instance = new NavX();
	public static GyroBase getInstance() { return instance; }
	
	 AHRS ahrs;
	
    // constructors
    public NavX() 
    {
    	ahrs = new AHRS(Constants.NAVX_PORT, Constants.NAVX_UPDATE_RATE);
    }
	
	/**
	 * Returns heading for the GyroBase class.
	 *
	 */
	public double getHeadingDeg() {
		return ahrs.getAngle();			 
	}
	
}
