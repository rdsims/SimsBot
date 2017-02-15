package org.team686.lib.util;

import java.text.DecimalFormat;

import org.mini2Dx.gdx.math.*;



/**
 * A class that stores the pose an object
 * The pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading
 * The reference coordinate system for the pose is not defined in this class.  The caller must keep track of it.
 */
public class Pose implements Interpolable<Pose>
{
	// using floats to take advantage of libgdx's speed optimizations
	
    private Vector2 position;	// position (x,y) in inches
    private float   heading;	// heading in radians

    static public Pose DEFAULT = new Pose(0f, 0f, 0f);

    public Pose() 
    {
    	this(0f, 0f, 0f);
    }

    
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

    public Pose(Vector2 _position, float _heading) 
    {
    	this.position = _position.cpy();
    	this.heading  = _heading;
    }

    public Pose(Vector2 _position, double _heading) 
    {
    	this(_position, (float)_heading);
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
    public Pose addHeadingRad(double _thetaRad)
    {
    	heading += _thetaRad;
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

    
    
    /*
     * Apply rigid transform to Pose
     */
    public Pose transformBy(RigidTransform _T)
    {
    	// use identical function in RigidTransform
    	RigidTransform pose = new RigidTransform(this.getPosition(), this.getHeadingRad());
    	
    	RigidTransform newPose = pose.transformBy(_T);
    	
    	return new Pose(newPose.getTranslation(), newPose.getRotationRad());
    }
    
    
     // Linear interpolation of poses
    @Override
    public Pose interpolate(Pose that, double u)
    {
    	Pose iPose;
    	
        if (u <= 0)
            iPose = new Pose(this);	
        else if (u >= 1) 
            iPose = new Pose(that);
        else
        {
        	Vector2 iPosition = position.lerp(that.position, (float)u);	// use Vector2's linear interpolation method
        	double  iHeading  = (heading * (1-u)) + (that.heading * u);	// linear interpolation of heading
        	iPose = new Pose(iPosition, iHeading); 
        }        
        return iPose;
    }

    
    
    @Override
    public String toString() 
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        String ret = this.position.toString() + ", H:" + fmt.format(this.getHeadingDeg());
        return ret;
    }
}


