package org.team686.lib.util;

import org.team686.lib.util.Interpolable;

public class Vector implements Interpolable<Vector>
{
	protected double x;
	protected double y;
	
	// constructors
	public Vector()
	{
		x = 0;
		y = 0;
	}

	public Vector(double _x, double _y)
	{
		x = _x;
		y = _y;
	}
	
	public Vector(Vector _v)
	{
		x = _v.x;
		y = _v.y;
	}
	
	// sets/gets
	public void setX(double x) { this.x = x; }
	public void setY(double y) { this.y = y; }
	
    public double getX() { return x; }
	public double getY() { return y; }

	
	// radians<-->degrees conversions
	static public final double radiansToDegrees = 180 / Math.PI;
	static public final double radDeg = radiansToDegrees;
	static public final double degreesToRadians = Math.PI / 180;
	static public final double degRad = degreesToRadians;
    
	
	
	// arithmetic
	public Vector add(Vector _v) { return new Vector(x + _v.x, y + _v.y); }
    public Vector sub(Vector _v) { return new Vector(x - _v.x, y - _v.y); }
    public Vector neg() { return new Vector(-x, -y); }

    // absolute value (length) of vector
    public double abs()
    {
    	return Math.hypot(x, y);
    }
    
    // angle from origin to this
    public double angle()
    {
    	return Math.atan2(y, x);
    }

    // angle from this to _v
    public double angle(Vector _v)
    {
    	double dx = _v.x - this.x;
    	double dy = _v.y - this.y;
    	return Math.atan2(dy, dx);
    }

    // vector cross product
	public double cross(Vector _v)
	{
		return (this.x * _v.y - this.y * _v.x);
	}
    
    // distance from this to _v
    public double distance(Vector _v)
    {
    	double dx = _v.x - this.x;
    	double dy = _v.y - this.y;
    	return Math.hypot(dx, dy);
    }

    public double distanceSqr(Vector _v)
    {
    	double dx = _v.x - this.x;
    	double dy = _v.y - this.y;
    	return (dx*dx + dy*dy);
    }
    
    // vector dot product
	public double dot(Vector _v)
	{
		return (this.x * _v.x + this.y * _v.y);
	}
    
    /*
     *  perform exponential filtering on position
     *  alpha is the filtering coefficient, 0<alpha<<1
     *  result will converge 63% in 1/alpha timesteps
     *                       86% in 2/alpha timesteps
     *                       95% in 3/alpha timesteps
     */
    public Vector expAverage(Vector _v, double _alpha)
    {
    	// exponential averaging
    	// u = (1-a)*u + a*v
    	double ax = (1-_alpha)*x + _alpha*_v.x;
    	double ay = (1-_alpha)*y + _alpha*_v.y;
    	return new Vector(ax, ay);
    }
    
    // absolute value (length) of vector
    public double length()
    {
    	return Math.hypot(x, y);
    }
    
    static public Vector magnitudeAngle(double _mag, double _angleRad)
    {
    	return new Vector(_mag*Math.cos(_angleRad), _mag*Math.sin(_angleRad));

    }
    
	// Rotates Vector by the given angle
	public Vector rotate(double _angleRad)
	{
		double cos = Math.cos(_angleRad);
		double sin = Math.sin(_angleRad);

		double x = this.x * cos - this.y * sin;
		double y = this.x * sin + this.y * cos;
		return new Vector(x,y);
	}
    
	// Rotates Vector by the given angle
	public Vector rotateDeg(double _angleDeg)
	{
		return this.rotate(_angleDeg * degreesToRadians);
	}
    

    // linearly interpolate between this (for u=0) and that (for u=1)
    @Override
    public Vector interpolate(Vector that, double _u)
    {
    	double u = _u;
        if (u < 0)
        	u = 0;
        if (u > 1)
        	u = 1;
        
        double x = this.x + u*(that.x - this.x);
        double y = this.y + u*(that.y - this.y);
		return new Vector(x,y);
    }
    
	
	@Override
	public String toString ()
	{
		return String.format("(% 7.3f, % 7.3f)", x, y);
	}
    
}
