package org.team686.lib.util;

import java.text.DecimalFormat;




/**
 * Performs translations and rotations on Vector2 and RigidTransform objects.
 */
public class RigidTransform implements Interpolable<RigidTransform>
{
    private Vector translation;		// translation(x,y) in inches
    private double rotation;		// rotation in radians

    // constructors
    public RigidTransform() 
    {
    	this(0, 0, 0);
    }

    public RigidTransform(double _x, double _y) 
    {
    	this(_x, _y, 0.0);
    }

    public RigidTransform(double _x, double _y, double _rotationRad) 
    {
        translation = new Vector(_x,_y);
        rotation  = _rotationRad;
    }

    public RigidTransform(Vector _translation, double _rotationRad) 
    {
    	translation = new Vector(_translation);
		rotation  = _rotationRad;
    }

    public RigidTransform(RigidTransform that) 
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
    public Vector getTranslation() { return translation; }
    public double getRotationRad() { return rotation; }
    public double getRotationDeg() { return rotation * radiansToDegrees; }
    public Vector getRotationUnitVector() { return new Vector(Math.cos(rotation), Math.sin(rotation)); }
    
    
    // add performs vector translation.  The original rotation is not changed
    public RigidTransform add(Vector _translation)
    {
    	return new RigidTransform(translation.add(_translation), rotation);
    }
    
    // sub performs vector translation.  The original rotation is not changed
    public RigidTransform sub(Vector _translation)
    {
    	return new RigidTransform(translation.sub(_translation), rotation);
    }
    
    // performs rotation.
    public RigidTransform rotateRad(double _thetaRad)
    {
    	return new RigidTransform(translation.rotate(_thetaRad), rotation+_thetaRad);
    }


    
    
    /*
     * Transforming this RigidTransform means first translating by
     * other.translation and then rotating by other.rotation
     */
    public RigidTransform transformBy(RigidTransform _that)
    {
    	// rotate T's translation by our rotation 
    	RigidTransform T = _that.rotateRad(this.rotation);

    	// apply translation
    	T = this.add(T.translation);
    	
    	// adjust rotation
    	double R = this.rotation + _that.rotation;
    	
    	return new RigidTransform(T.translation, R);
    }
    
    
    /**
     * Vector rotation
     */
    public RigidTransform rotateRad(float _rotationRad)
    {
    	// caution: rotation affects this 
    	translation.rotate(_rotationRad);			// rotate translation by _rotationRad
    	// this.rotation is not affected
    	return this; 
    }
    
    /**
     * The inverse of this transform "undoes" the effect of translating by this
     * transform.
     */
    public RigidTransform inverse()
    {
    	RigidTransform T = new RigidTransform(-this.translation.x, -this.translation.y, 0);	// invert translation
    	T.rotateRad(-this.rotation);														// rotate translation by inverse of rotation
    	T.rotation = -this.rotation;														// invert rotation
    	return T;												
    }
    
    
    
    
     // Linear interpolation of RigidTransforms
    @Override
    public RigidTransform interpolate(RigidTransform _that, double _u)
    {
     	double u = _u;
        if (u < 0)
            u = 0;	
        if (u > 1) 
            u = 1;
        
    	Vector iTranslation = translation.interpolate(_that.translation, u);	// interpolate position
    	double  iRotation = this.rotation + u*(this.rotation - _that.rotation);	// interpolate heading
    	 
        return new RigidTransform(iTranslation, iRotation);
    }

    
    
    @Override
    public String toString() 
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        String ret = this.translation.toString() + ", R:" + fmt.format(this.getRotationDeg());
        return ret;
    }
}


