package org.usfirst.frc.team686.lib.util;

/**
 * A class that stores the pose an object
 * The pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading, theta
 * The reference coordinate system for the pose is not defined in this class.  The caller must keep track of it.
 */
public class Pose {
    private double x;		// x coordinate (in inches)
    private double y;		// y coordinate (in inches)
    private double theta;	// heading (in radians)

    public Pose(double _x, double _y) 
    {
        set(_x, _y, 0.0);
    }

    public Pose(double _x, double _y, double _theta) 
    {
    	set(_x, _y, _theta);
    }

    public static Pose fromDistanceHeadingRadians(double distance, double headingRadians)
    {
        return new Pose(distance*Math.cos(headingRadians), distance*Math.sin(headingRadians), headingRadians);	// arbitrarily setting heading to headingRadians
    }
    
    public void set(double _x, double _y, double _theta)
    {
        x = _x;
        y = _y;
        theta = _theta;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getTheta() { return theta; }
    
    
    // add performs vector translation.  The original heading is not changed
    public Pose add(Pose that)
    {
    	return new Pose(x+that.x, y+that.y, theta);
    }
    
    // sub performs vector translation.  The original heading is not changed
    public Pose sub(Pose that)
    {
    	return new Pose(x-that.x, y-that.y, theta);
    }
    
    // performs rotation.  The original (x,y) location is not changed
    public Pose rotateRadians(double rotationRadians)
    {
    	return new Pose(x, y, theta+rotationRadians);
    }

    // performs rotation.  The original (x,y) location is not changed
    public Pose rotateDegrees(double rotationDegrees)
    {
    	return this.rotateRadians(rotationDegrees * Math.PI/180.0);
    }
    
    public double distance()
    {
    	return distance(new Pose(0.0, 0.0));
    }

    public double distance(Pose that)
    {
    	double dx = that.x - this.x;
    	double dy = that.y - this.y;
    	return Math.sqrt(dx*dx+dy*dy);
    }

    // heading from this to that in radians
    public double headingRadians(Pose that)
    {
    	double dx = that.x - this.x;
    	double dy = that.y - this.y;
    	return Math.atan2(dy, dx);
    }

    // heading from this to that in degrees
    public double headingDegrees(Pose that)
    {
    	return this.headingRadians(that) * 180.0/Math.PI;
    }
    
    // perform exponential filtering on position
    // alpha is the filtering coefficient, 0<alpha<<1
    // result will converge 63% in 1/alpha timesteps
    //                      86% in 2/alpha timesteps
    //                      95% in 3/alpha timesteps
    public Pose filterPosition(Pose that, double alpha)
    {
    	return new Pose((1-alpha)*x + alpha*that.x,
    			        (1-alpha)*y + alpha*that.y,
    			        theta);
    }
    
    
    @Override
    public String toString() {
        return "X: " + x + ", Y: " + y + ", H: " + theta;
    }
}


