package org.team686.lib.util;

/**
 * Take-back-half speed control system
 * Improves on bang-bang controller by converging on steady-state value
 *
 * For information, see:
 * 		http://www.chiefdelphi.com/forums/showthread.php?threadid=105965">http://www.chiefdelphi.com/forums/showthread.php?threadid=105965
 * 		http://www.chiefdelphi.com/media/papers/2674?">http://www.chiefdelphi.com/media/papers/2674
 * 
 */
public class TakeBackHalfSpeedController
{
    private double targetRpm = 0;		// target speed, in RPM
    private double motorPower = 0;		// The ratio (0-1) of motor power that is being applied (assumes use in %Vbus mode)
    
    private double lastError = 0;		// error from last loop
    private double tbh = 0;				// take-back-half variable
    
    private static double gain;			// loop gain (should be <=1).  Increase to increase convergence time, decrease to reduce final oscillation
    private static double maxRpm;		// approximate maximum RPM, used for spinup optimization

    public TakeBackHalfSpeedController(double _targetRpm, double _gain, double _maxRpm)
    {
    	maxRpm = _maxRpm;
    	gain = _gain;
    	setTargetRpm(_targetRpm);
    }

    public synchronized void setGain(double _gain) 
    {
    	gain = _gain;
    }

    public synchronized void setTargetRpm(double _newRpm) 
    {
        //Set up values for optimized spin-up to the target
        if (targetRpm < _newRpm)
            lastError = 1;
        else if (targetRpm > _newRpm) 
            lastError = -1;

        tbh = (2 * (_newRpm / maxRpm)) - 1;		// set to optimize spin-up time (minimize unnecessary zero-crossings)
        
        targetRpm = _newRpm;
    }

    protected synchronized double run(double _measuredRpm)
    {
        double error = targetRpm - _measuredRpm;
		
        motorPower += gain * error;
        motorPower = clamp(motorPower);

        //If the error has changed in sign since the last processing
        if (isPositive(lastError) != isPositive(error))
        {
            motorPower = 0.5 * (motorPower + tbh);
            tbh = motorPower;

            lastError = error;
        }

        return motorPower;
    }

    private static double clamp(double _input) 
    {
        if (_input > 1)
            return 1;
        
        if (_input < -1)
            return -1;

        return _input;
    }

    private static boolean isPositive(double _input) 
    {
        return _input > 0;
    }
}