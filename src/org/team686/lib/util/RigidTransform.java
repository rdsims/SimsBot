package org.team686.lib.util;

import java.text.DecimalFormat;

import org.mini2Dx.gdx.math.*;



/**
 * Performs translations and rotations on Vector2 and RigidTransform objects.
 */
public class RigidTransform implements Interpolable<RigidTransform>
{
	// using floats to take advantage of libgdx's speed optimizations
	
    private Vector2 translation;	// translation(x,y) in inches
    private float   rotation;		// rotation in radians

    public RigidTransform() 
    {
    	this(0f, 0f, 0f);
    }

    
    public RigidTransform(float _x, float _y) 
    {
    	this(_x, _y, 0.0f);
    }

    public RigidTransform(float _x, float _y, float _thetaRad) 
    {
        translation = new Vector2(_x,_y);
        rotation  = _thetaRad;
    }

    public RigidTransform(double _x, double _y) 
    {
    	this((float)_x, (float)_y, 0.0f);
    }

    public RigidTransform(double _x, double _y, double _thetaRad) 
    {
    	this((float)_x, (float)_y, (float)_thetaRad);
    }

    public RigidTransform(Vector2 _translation, float _rotation) 
    {
    	this.translation = _translation.cpy();
    	this.rotation  = _rotation;
    }

    public RigidTransform(Vector2 _translation, double _rotation) 
    {
    	this(_translation, (float)_rotation);
    }

    public RigidTransform(RigidTransform that) 
    {
    	this.translation = that.translation.cpy();
    	this.rotation  = that.rotation;
    }

    
    public static RigidTransform fromMagnitudeAngleRad(float _rho, float _thetaRad)
    {
    	RigidTransform RigidTransform = new RigidTransform(_rho, 0.0f, _thetaRad);	// arbitrarily setting rotation to thetaRad
    	RigidTransform.translation.rotateRad(_thetaRad);
    	return RigidTransform;
    }
    
    public void set(float _x, float _y, float _thetaRad)
    {
    	this.translation.set(_x, _y);
        rotation = _thetaRad;
    }

    public float getX() { return translation.x; }
    public float getY() { return translation.y; }
    public Vector2 getTranslation() { return translation; }
    public float getRotationRad() { return rotation; }
    public float getRotationDeg() { return rotation * MathUtils.radiansToDegrees; }
    public Vector2 getRotationVector2() { return new Vector2((float)Math.cos(rotation), (float)Math.sin(rotation)); }


    /*
     * Transforming this RigidTransform2d means first translating by
     * other.translation and then rotating by other.rotation
     */
    public RigidTransform transformBy(RigidTransform _other)
    {
    	// make copies, so we don't affect the references
    	RigidTransform newT = new RigidTransform(this);
    	RigidTransform other = new RigidTransform(_other);
    	
    	// rotate T's translation by our heading 
    	other.rotateRad(newT.getRotationRad());
    	
    	// apply translation
    	newT.translation.add(other.getTranslation());
    	
    	// adjust rotation
    	newT.rotation += other.getRotationRad();
    	
    	return newT;
    }
    
    
    /**
     * Vector rotation
     */
    public RigidTransform rotateRad(float _rotationRad)
    {
    	// caution: rotation affects this 
    	translation.rotateRad(_rotationRad);			// rotate translation by _rotationRad
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
    public RigidTransform interpolate(RigidTransform that, double u)
    {
    	RigidTransform T;
    	
        if (u <= 0)
        	T = new RigidTransform(this);	
        else if (u >= 1) 
        	T = new RigidTransform(that);
        else
        {
        	Vector2 iTranslation = translation.lerp(that.translation, (float)u);	// use Vector2's linear interpolation method
        	double  iRotation  = (rotation * (1-u)) + (that.rotation * u);	// linear interpolation of rotation
        	T = new RigidTransform(iTranslation, iRotation); 
        }        
        return T;
    }

    
    
    @Override
    public String toString() 
    {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        String ret = this.translation.toString() + ", R:" + fmt.format(this.getRotationDeg());
        return ret;
    }
}


