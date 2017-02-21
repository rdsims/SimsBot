package org.team686.lib.util;

/**
 * Contains basic functions that are used often.
 */
public class Util 
{
    /** Prevent this class from being instantiated. */
    private Util() {}

    // limits output to be in the range [lowerLimit, upperLimit]
    public static double limit(double _in, double _lowerLimit, double _upperLimit) 
    {
    	double out = _in;
    	if (out < _lowerLimit)
    		out = _lowerLimit;
    	if (out > _upperLimit)
    		out = _upperLimit;
    	return out;
    }

    // limits output to be in the range +/-limit
    public static double limit(double _in, double _limit) 
    {
    	return limit(_in, -_limit, +_limit);
    }

}
