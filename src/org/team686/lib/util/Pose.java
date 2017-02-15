package org.team686.lib.util;

import java.text.DecimalFormat;

import org.mini2Dx.gdx.math.*;
import org.team686.lib.util.RigidTransform2d.Delta;

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

    static public Pose DEFAULT = new Pose(0f, 0f, 0f);
    
    public Pose(float _x, float _y) 
    {
    	this(_x, _y, 0.0f);
    }

    public Pose(float _x, float _y, float _thetaRad) 
    {
        position = new Vector2(_x,_y);
        heading  = _thetaRad;
    }

    public Pose(double _x, double _y) 
    {
    	this((float)_x, (float)_y, 0.0f);
    }

    public Pose(double _x, double _y, double _thetaRad) 
    {
    	this((float)_x, (float)_y, (float)_thetaRad);
    }

    public Pose(Pose that) 
    {
    	this.position = that.position.cpy();
    	this.heading  = that.heading;
    }

    
    public static Pose fromMagnitudeAngleRad(float _rho, float _thetaRad)
    {
    	Pose pose = new Pose(_rho, 0.0f, _thetaRad);	// arbitrarily setting heading to thetaRad
    	pose.position.rotateRad(_thetaRad);
    	return pose;
    }
    
    public void set(float _x, float _y, float _thetaRad)
    {
    	this.position.set(_x, _y);
        heading = _thetaRad;
    }

    public float getX() { return position.x; }
    public float getY() { return position.y; }
    public Vector2 getPosition() { return position; }
    public float getHeadingRad() { return heading; }
    public float getHeadingDeg() { return heading * MathUtils.radiansToDegrees; }
    public Vector2 getHeadingVector2() { return new Vector2((float)Math.cos(heading), (float)Math.sin(heading)); }
    
    
    // add performs vector translation.  The original heading is not changed
    public Pose add(Vector2 v)
    {
    	position.add(v);
    	return this;
    }
    
    // adjust Heading
    public double addHeadingRad(double _thetaRad)
    {
    	heading += _thetaRad;
    	return heading;
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
    
    
    
    public static class Delta
    {
        public final double dDistance;
        public final double dHeadingRad;

        public Delta(double _dDistance, double _dHeadingRad)
        {
            dDistance = _dDistance;
            dHeadingRad = _dHeadingRad;
        }
    }
    
    
    /*
     * Obtain a new Pose from travel along a constant curvature path.
     */
    public Pose travelArc(Delta delta)
    {
		double D = delta.dDistance;				// distance traveled = arc-length of circle
		double L = D;							// chord-length
		
		double dTheta = delta.dHeadingRad;
		if (dTheta > 1e-9)
			L = 2*D*Math.sin(dTheta/2)/dTheta;			// chord-length given change in heading
				
		double avgHeading = heading + dTheta/2;			// mean of current and final headings

		// update pose
		Pose newPose = new Pose(this);										// copy current pose
		Vector2 deltaPosition = Util.fromMagnitudeAngleRad(L, avgHeading);	// calculate change in position
		newPose.add(deltaPosition);											// update position							
		newPose.addHeadingRad(dTheta);										// update heading
		return newPose;														// return new pose
    }

    
    
    @Override
    public String toString() 
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        String ret = this.position.toString() + ", H:" + fmt.format(this.getHeadingDeg());
        return ret;
    }
}


