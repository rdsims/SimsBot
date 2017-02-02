package org.team686.lib.util;

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
    

    
    
    
    
    /*
     * Helper functions for Vector2
     */
    
	// constructor that casts from double to float
	public Vector2 Vector2(double _x, double _y)
	{
		return new Vector2((float)_x, (float)_y);
	}
    
	// additional magnitude/phase constructors for Vector2
    public static Vector2 fromMagnitudeAngleRad(double _rho, double _angleRad)
    {
    	return fromMagnitudeAngleRad( (float)_rho, (float)_angleRad );
    }

    public static Vector2 fromMagnitudeAngleRad(float _rho, float _angleRad)
    {
    	Vector2 v = new Vector2(_rho, 0.0f);	// create vector with 0 angle
    	v.rotateRad(_angleRad);					// then rotate
    	return v;
    }
	
    /*
     *  perform exponential filtering on position
     *  alpha is the filtering coefficient, 0<alpha<<1
     *  result will converge 63% in 1/alpha timesteps
     *                       86% in 2/alpha timesteps
     *                       95% in 3/alpha timesteps
     */
    public static Vector2 expAverage(Vector2 u, Vector2 v, float alpha)
    {
    	// exponential averaging
    	// u = (1-a)*u + a*v
    	u.scl(1-alpha);
    	v.scl(alpha);
    	u.add(v);
    	return u;
    }
    
}
