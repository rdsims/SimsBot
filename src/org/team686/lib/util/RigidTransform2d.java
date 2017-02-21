package org.team686.lib.util;

import java.text.DecimalFormat;




/**
 * Performs translations and rotations on Vector2 and RigidTransform objects.
 */
public class RigidTransform2d implements Interpolable<RigidTransform2d>
{
    private Vector2d translation;		// translation(x,y) in inches
    private double rotation;		// rotation in radians

    // constructors
    public RigidTransform2d() 
    {
    	this(0, 0, 0);
    }

    public RigidTransform2d(double _x, double _y) 
    {
    	this(_x, _y, 0.0);
    }

    public RigidTransform2d(double _x, double _y, double _rotationRad) 
    {
        translation = new Vector2d(_x,_y);
        rotation  = _rotationRad;
    }

    public RigidTransform2d(Vector2d _translation, double _rotationRad) 
    {
    	translation = new Vector2d(_translation);
		rotation  = _rotationRad;
    }

    public RigidTransform2d(RigidTransform2d that) 
    {
    	this(that.translation, that.rotation);
    }

    
	/** multiply by this to convert from radians to degrees */
	static public final double radiansToDegrees = 180 / Math.PI;
	static public final double radDeg = radiansToDegrees;
	/** multiply by this to convert from degrees to radians */
	static public final double degreesToRadians = Math.PI / 180;
	static public final double degRad = degreesToRadians;
    
    public double getX() { return translation.x; }
    public double getY() { return translation.y; }
    public Vector2d getTranslation() { return translation; }
    public double getRotationRad() { return rotation; }
    public double getRotationDeg() { return rotation * radiansToDegrees; }
    public Vector2d getRotationUnitVector() { return new Vector2d(Math.cos(rotation), Math.sin(rotation)); }
    
    
    // add performs vector translation.  The original rotation is not changed
    public RigidTransform2d add(Vector2d _translation)
    {
    	return new RigidTransform2d(translation.add(_translation), rotation);
    }
    
    // sub performs vector translation.  The original rotation is not changed
    public RigidTransform2d sub(Vector2d _translation)
    {
    	return new RigidTransform2d(translation.sub(_translation), rotation);
    }
    
    // performs rotation.
    public RigidTransform2d rotateRad(double _thetaRad)
    {
    	return new RigidTransform2d(translation.rotate(_thetaRad), rotation+_thetaRad);
    }


    
    
    /*
     * Transforming this RigidTransform means first translating by
     * other.translation and then rotating by other.rotation
     */
    public RigidTransform2d transformBy(RigidTransform2d _that)
    {
    	// rotate T's translation by our rotation 
    	RigidTransform2d T = _that.rotateRad(this.rotation);

    	// apply translation
    	T = this.add(T.translation);
    	
    	// adjust rotation
    	double R = this.rotation + _that.rotation;
    	
    	return new RigidTransform2d(T.translation, R);
    }
    
    
    /**
     * Vector rotation
     */
    public RigidTransform2d rotateRad(float _rotationRad)
    {
    	return new RigidTransform2d(translation.rotate(_rotationRad), rotation);	// rotate translation by _rotationRad
    																			// this.rotation is not affected
    }
    
    /**
     * The inverse of this transform "undoes" the effect of translating by this
     * transform.
     */
    public RigidTransform2d inverse()
    {
    	RigidTransform2d T = new RigidTransform2d(-this.translation.x, -this.translation.y, 0);	// invert translation
    	T = T.rotateRad(-this.rotation);													// rotate translation by inverse of rotation
    	return T;												
    }
    
    
    
    
     // Linear interpolation of RigidTransforms
    @Override
    public RigidTransform2d interpolate(RigidTransform2d _that, double _u)
    {
     	double u = _u;
        if (u < 0)
            u = 0;	
        if (u > 1) 
            u = 1;
        
    	Vector2d iTranslation = translation.interpolate(_that.translation, u);	// interpolate position
    	double  iRotation = this.rotation + u*(_that.rotation - this.rotation);	// interpolate heading
    	 
        return new RigidTransform2d(iTranslation, iRotation);
    }

    
    
    @Override
    public String toString() 
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        String ret = this.translation.toString() + ", R:" + fmt.format(this.getRotationDeg());
        return ret;
    }
}


