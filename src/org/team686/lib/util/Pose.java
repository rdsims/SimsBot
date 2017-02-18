package org.team686.lib.util;

/**
 * A class that stores the pose an object
 * The pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading
 * The reference coordinate system for the pose is not defined in this class.  The caller must keep track of it.
 */
public class Pose implements Interpolable<Pose>
{
    private Vector position;	// position (x,y) in inches
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
        position = new Vector(_x,_y);
        heading  = _headingRad;
    }

    public Pose(Vector _position, double _headingRad) 
    {
    	position = new Vector(_position);
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
    	Vector p = new Vector(_rho, 0);
    	p.rotate(_thetaRad);
    	return new Pose(p, _thetaRad);		// arbitrarily setting heading to thetaRad
    }
    
    public double getX() { return position.x; }
    public double getY() { return position.y; }
    public Vector getPosition() { return position; }
    public double getHeadingRad() { return heading; }
    public double getHeadingDeg() { return heading * radiansToDegrees; }
    public Vector getHeadingUnitVector() { return new Vector(Math.cos(heading), Math.sin(heading)); }
    
    
    // add performs vector translation.  The original heading is not changed
    public Pose add(Vector _translation)
    {
    	return new Pose(position.add(_translation), heading);
    }
    
    // returns vector from that to this.position
    public Vector sub(Vector _translation)
    {
    	return position.sub(_translation);
    }
    
    // returns vector from that.position to this.position
    public Vector sub(Pose _that)
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
    public double distance(Vector _that)
    {
    	return position.distance(_that);
    }

    // heading from this to that in radians
    public double headingRad(Vector _that)
    {
    	return this.position.angle(_that);
    }

    // heading from this to that in degrees
    public double headingDeg(Vector _that)
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
    }
    
    
    /*
     * Obtain a new Pose from travel along a constant curvature path.
     */
    public Pose travelArc(Delta delta)
    {
		double D = delta.dDistance;				// distance traveled = arc-length of circle
		double L = D;							// chord-length
		
		double dTheta = delta.dHeading;
		if (Math.abs(dTheta) > 1e-9)
			L = 2*D*Math.sin(dTheta/2)/dTheta;			// chord-length given change in heading
				
		double avgHeading = heading + dTheta/2;			// mean of current and final headings

		// update pose
		Vector deltaPosition = Vector.magnitudeAngle(L, avgHeading);	// calculate change in position
		return new Pose(position.add(deltaPosition), heading+delta.dHeading);	// return new pose
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
    public Pose interpolate(Pose _that, double _u)
    {
    	double u = _u;
        if (u < 0)
            u = 0;	
        if (u > 1) 
            u = 1;
        
    	Vector iPosition = position.interpolate(_that.position, u);			// interpolate position
    	double  iHeading = this.heading + u*(_that.heading - this.heading);	// interpolate heading
    	 
        return new Pose(iPosition, iHeading);
    }

    
    
    @Override
    public String toString() 
    {
    	return String.format("%s, H: % 5.1f deg", position.toString(), getHeadingDeg());
    }
}


