package org.team686.lib.util;

/**
 * A class that stores the pose an object
 * The pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading
 * The reference coordinate system for the pose is not defined in this class.  The caller must keep track of it.
 */
public class Pose implements Interpolable<Pose>
{
    private Vector2d position;	// position (x,y) in inches
    private double heading;		// heading in radians

    // constructors
    public Pose() 
    {
    	this(0, 0, 0);
    }

    public Pose(double _x, double _y) 
    {
    	this(_x, _y, 0.0);
    }

    public Pose(double _x, double _y, double _headingRad) 
    {
        position = new Vector2d(_x,_y);
        heading  = _headingRad;
    }

    public Pose(Vector2d _position, double _headingRad) 
    {
    	position = new Vector2d(_position);
		heading  = _headingRad;
    }

    public Pose(Pose that) 
    {
    	this(that.position, that.heading);
    }

    
	/** multiply by this to convert from radians to degrees */
	static public final double radiansToDegrees = 180 / Math.PI;
	static public final double radDeg = radiansToDegrees;
	/** multiply by this to convert from degrees to radians */
	static public final double degreesToRadians = Math.PI / 180;
	static public final double degRad = degreesToRadians;
    
    public static Pose fromMagnitudeAngleRad(double _rho, double _thetaRad)
    {
    	Vector2d p = new Vector2d(_rho, 0);
    	p.rotate(_thetaRad);
    	return new Pose(p, _thetaRad);		// arbitrarily setting heading to thetaRad
    }
    
    public double getX() { return position.x; }
    public double getY() { return position.y; }
    public Vector2d getPosition() { return position; }
    public double getHeadingRad() { return heading; }
    public double getHeadingDeg() { return heading * radiansToDegrees; }
    public Vector2d getHeadingUnitVector() { return new Vector2d(Math.cos(heading), Math.sin(heading)); }
    
    
    // add performs vector translation.  The original heading is not changed
    public Pose add(Vector2d _translation)
    {
    	return new Pose(position.add(_translation), heading);
    }
    
    // returns vector from that to this.position
    public Vector2d sub(Vector2d _translation)
    {
    	return position.sub(_translation);
    }
    
    // returns vector from that.position to this.position
    public Vector2d sub(Pose _that)
    {
    	return this.sub(_that.position);
    }
    
    // adjust heading without changing position
    // (use rotateRad to rotate about origin)
    public Pose turnRad(double _thetaRad)
    {
    	return new Pose(position, heading+_thetaRad);
    }
    
    // rotates position about origin, and adjusts heading by _thetaRad
    public Pose rotateRad(double _thetaRad)
    {
    	return new Pose(position.rotate(_thetaRad), heading+_thetaRad);
    }

    // get distance from this pose to vector v
    public double distance(Vector2d _that)
    {
    	return position.distance(_that);
    }

    // heading from this to that in radians
    public double headingRad(Vector2d _that)
    {
    	return this.position.angle(_that);
    }

    // heading from this to that in degrees
    public double headingDeg(Vector2d _that)
    {
    	return headingRad(_that) * radiansToDegrees;
    }
    
    
    
    public static class Delta
    {
        public final double dDistance;		// change in position in inches
        public final double dHeading;		// change in heading in radians

        public Delta(double _dDistance, double _dHeadingRad)
        {
            dDistance = _dDistance;
            dHeading = _dHeadingRad;
        }
        
        // TODO: Delta constructor from speed/curvature
    }
    
    
    /*
     * Apply rigid transform to Pose
     */
    public Pose transformBy(RigidTransform2d _T)
    {
    	// use identical function in RigidTransform
    	RigidTransform2d pose = new RigidTransform2d(this.getPosition(), this.getHeadingRad());
    	
    	RigidTransform2d newPose = pose.transformBy(_T);
    	
    	return new Pose(newPose.getTranslation(), newPose.getRotationRad());
    }
    
    
     // Linear interpolation of poses
    @Override
    public Pose interpolate(Pose _that, double _u)
    {
    	double u = _u;
        if (u < 0)
            u = 0;	
        if (u > 1) 
            u = 1;
        
    	Vector2d iPosition = position.interpolate(_that.position, u);			// interpolate position
    	double  iHeading = this.heading + u*(_that.heading - this.heading);	// interpolate heading
    	 
        return new Pose(iPosition, iHeading);
    }

    
    
    @Override
    public String toString() 
    {
    	return String.format("%s, H: % 5.1f deg", position.toString(), getHeadingDeg());
    }
}


