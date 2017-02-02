package org.team686.lib.util;

/**
 * A class that stores the pose of the robot
 * The robots pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading, theta
 */
public class RobotPose {
    public double x;		// x coordinate (in inches)
    public double y;		// y coordinate (in inches)
    public double theta;	// heading (in radians)

    public RobotPose(double _x, double _y) 
    {
        this.set(_x, _y, 0.0);
    }

    public RobotPose(double _x, double _y, double _theta) 
    {
    	this.set(_x, _y, _theta);
    }

    public void set(double _x, double _y, double _theta)
    {
        this.x = _x;
        this.y = _y;
        this.theta = _theta;
    }
    
    @Override
    public String toString() {
        return "X: " + x + ", Y: " + y + ", H: " + theta;
    }
}
