package org.team686.lib.util;

import java.util.List;
import java.util.Optional;

/**
 * Contains basic functions that are used often.
 */
public class Util 
{
    /** Prevent this class from being instantiated. */
    private Util() {}

    protected static final double kEpsilon = 1E-9;

    
    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double val, double limit) 
    {
        return (Math.abs(val) < limit) ? val : Math.signum(val) * limit;
    }

    public static double limit(double _in, double _lowerLimit, double _upperLimit) 
    {
    	double out = _in;
    	if (out < _lowerLimit)
    		out = _lowerLimit;
    	if (out > _upperLimit)
    		out = _upperLimit;
    	return out;
    }

    
    public static class ClosestPointOnSegment 
    {
        public double index; 			// Index of the point on the path segment (not clamped to [0, 1])
        public double clampedIndex; 	// As above, but clamped to [0, 1]
        public Vector2d point; 	// The result of interpolate(clamped_index)
        public double distance; 		// The distance from closest_point to the query point
    }
    
    // find closest point on segment AB to point P
    public static ClosestPointOnSegment getClosestPointOnSegment(Vector2d _a, Vector2d _b, Vector2d _p) 
    {
    	ClosestPointOnSegment rv = new ClosestPointOnSegment();
    	
    	Vector2d ab = _b.sub(_a);				// line segment AB
    	double abLengthSqr = ab.lengthSqr();	// |AB|^2
    	
        if (abLengthSqr < kEpsilon)
        {
        	// segment is very small.  return A (which is near B)
            rv.index = rv.clampedIndex = 0.0;
            rv.point = new Vector2d(_a);
        }
        else
        {
        	Vector2d ap = _p.sub(_a);
        	double dot = ap.dot(ab);
            rv.index = dot / abLengthSqr;						// index = |AP|/|AB| cos(angle between AP & AB)   
            rv.clampedIndex = Util.limit(rv.index, 0.0, 1.0);	// clamp in case nearest point is outside segment
            rv.point = _a.interpolate(_b, rv.index);			// point on AB closest to P
        } 

        rv.distance = _p.distance(rv.point);
        return rv;
    }

    
    public static Optional<Vector2d[]> getLineCircleIntersection(Vector2d _p1, Vector2d _p2, Vector2d _center, double _radius)
    {
    	// points of intersection are at 
    	// x = {  D*dy +/- sign(dy)*dx*sqrt(R^2*dr^2-D^2) } / dr^2
    	// y = { -D*dx +/- sign(dy)*   sqrt(R^2*dr^2-D^2) } / dr^2
    	// where D is the determinant

    	// shift everything so that center of circle is at (0,0)
    	Vector2d p1 = new Vector2d(_p1).sub(_center); 
    	Vector2d p2 = new Vector2d(_p2).sub(_center);
    	
    	Vector2d d = p2.sub(p1);
    	double dx = d.getX();				// dx = x2-x1
    	double dy = d.getY();				// dy = y2-y1
    	double drSqr = d.lengthSqr();		// drSqr = dx^2 + dy^2
    	
    	double det = p1.cross(p2);			// D = determinant = x1*y2 - x2*y1  
    	double disc = (_radius * _radius * drSqr - det * det);	// discriminant = (R^2*dr^2-D^2)		
    	
        if (disc < 0) 
        {
            // no real solutions --> no intersection
            return Optional.empty();
        }

        if (disc == 0)
        {
        	// single solution
        	Vector2d[] soln = new Vector2d[1];
        	soln[0] = new Vector2d( det * dy / drSqr, -det * dx / drSqr);
        	soln[0] = soln[0].add(_center);
        	return Optional.of(soln);
        }

        // discriminant > 0: two solutions
        double sqrtDisc = Math.sqrt(disc);
        double sy = (dy<0 ? -1 : 1);

    	Vector2d[] soln = new Vector2d[2];
    	soln[0] = new Vector2d( (det * dy + sy * dx * sqrtDisc) / drSqr, (-det * dx + sy * dy * sqrtDisc) / drSqr);
    	soln[1] = new Vector2d( (det * dy - sy * dx * sqrtDisc) / drSqr, (-det * dx - sy * dy * sqrtDisc) / drSqr);
    	soln[0] = soln[0].add(_center);
    	soln[1] = soln[1].add(_center);
    	return Optional.of(soln);
    }
    
    
    
    public static String joinStrings(String delim, List<?> strings) 
    {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) 
        {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) 
            {
                sb.append(delim);
            }
        }
        return sb.toString();
    }
    
}
