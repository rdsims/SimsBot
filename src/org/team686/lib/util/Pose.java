package org.team686.lib.util;

import org.mini2Dx.gdx.math.*;

/**
 * A class that stores the pose an object
 * The pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading
 * The reference coordinate system for the pose is not defined in this class.  The caller must keep track of it.
 */
public class Pose 
{
	// using floats to take advantage of libgdx's speed optimizations
	
    private Vector2 position;	// position (x,y) in inches
    private float   heading;	// heading in radians

    public Pose(float _x, float _y) 
    {
        position = new Vector2(_x,_y);
        heading  = 0.0f;
    }

    public Pose(float _x, float _y, float _thetaRad) 
    {
        position = new Vector2(_x,_y);
        heading  = _thetaRad;
    }

    public Pose(double _x, double _y) 
    {
    	new Pose((float)_x, (float)_y);
    }

    public Pose(double _x, double _y, double _thetaRad) 
    {
    	new Pose((float)_x, (float)_y, (float)_thetaRad);
    }

    public static Pose fromMagnitudeAngleRad(float _rho, float _thetaRad)
    {
    	Pose pose = new Pose(_rho, 0.0f, _thetaRad);	// arbitrarily setting heading to thetaRad
    	pose.position.rotateRad(_thetaRad);
    	return pose;
    }
    
    public void set(float _x, float _y, float _theta)
    {
    	position.set(_x, _y);
        heading = _theta;
    }

    public float getX() { return position.x; }
    public float getY() { return position.y; }
    public Vector2 getPosition() { return position; }
    public float getHeadingRad() { return heading; }
    public float getHeadingDeg() { return heading * MathUtils.radiansToDegrees; }
    
    
    // add performs vector translation.  The original heading is not changed
    public Pose add(Vector2 v)
    {
    	position.add(v);
    	return this;
    }
    
    // sub performs vector translation.  The original heading is not changed
    public Pose sub(Vector2 v)
    {
    	position.sub(v);
    	return this;
    }
    
    // performs rotation.  The original (x,y) location is not changed
    public Pose rotateRad(float radians)
    {
    	position.rotateRad(radians);
    	return this;
    }

    // performs rotation.  The original (x,y) location is not changed
    public Pose rotateDeg(float degrees)
    {
    	position.rotate(degrees);
    	return this;
    }
    
    public float distance(Vector2 v)
    {
    	return position.dst(v);
    }

    // heading from this to that in radians
    public double headingRadians(Vector2 that)
    {
    	Vector2 delta = that.sub(position);		// vector from this to that
    	return delta.angleRad();
    }

    // heading from this to that in degrees
    public double headingDegrees(Vector2 that)
    {
    	Vector2 delta = that.sub(position);		// vector from this to that
    	return delta.angle();
    }
    
    @Override
    public String toString() {
        return "X: " + position.x + ", Y: " + position.y + ", H: " + heading;
    }
}

