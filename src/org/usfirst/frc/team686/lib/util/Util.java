package org.usfirst.frc.team686.lib.util;

import java.util.List;
import org.mini2Dx.gdx.math.*;

/**
 * Contains basic functions that are used often.
 */
public class Util 
{
    /** Prevent this class from being instantiated. */
    private Util() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double val, double limit) 
    {
        return (Math.abs(val) < limit) ? val : Math.signum(val) * limit;
    }

    public static String joinStrings(String delim, List<?> strings) 
    {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) 
        {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) 
            {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    // additional constructors for Vector2
    public static Vector2 fromMagnitudeAngleRad(double _rho, double _angleRad)
    {
    	return fromMagnitudeAngleRad( (float)_rho, (float)_angleRad );
    }

    public static Vector2 fromMagnitudeAngleRad(float _rho, float _angleRad)
    {
    	Vector2 v = new Vector2(_rho, 0.0f);
    	v.rotateRad(_angleRad);
    	return v;
    }
    
}
