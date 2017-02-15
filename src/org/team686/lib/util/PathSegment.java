package org.team686.lib.util;

import org.mini2Dx.gdx.math.Vector2;

/**
 * A PathSegment consists of two Vector2 objects (the start and end
 * points) as well as the speed of the robot.
 *
 */
public class PathSegment {
    protected static final double kEpsilon = 1E-9;

    public static class Sample
    {
        public final Vector2 position;
        public final double speed;

        public Sample(Vector2 translation, double speed) 
        {
            this.position = translation.cpy();
            this.speed = speed;
        }
    }

    protected double mSpeed;
    protected Vector2 mStart;
    protected Vector2 mEnd;
    protected Vector2 mStartToEnd; // pre-computed for efficiency
    protected double mLength; // pre-computed for efficiency

    public static class ClosestPointReport 
    {
        public double index; // Index of the point on the path segment (not
                             // clamped to [0, 1])
        public double clamped_index; // As above, but clamped to [0, 1]
        public Vector2 closest_point; // The result of
                                            // interpolate(clamped_index)
        public double distance; // The distance from closest_point to the query
                                // point
    }

    public PathSegment(Vector2 start, Vector2 end, double speed) 
    {
        mEnd = end.cpy();	// local copy
        mSpeed = speed;
        updateStart(start);
    }

    public void updateStart(Vector2 new_start)
    {
        mStart = new_start.cpy();		
        Vector2 tempEnd = mEnd.cpy();	// local copy, so mEnd isn't modified
        mStartToEnd = tempEnd.sub(mStart);
        mLength = mStartToEnd.len();
    }

    public double  getSpeed()  { return mSpeed; }
    public Vector2 getStart()  { return mStart; }
    public Vector2 getEnd()    { return mEnd; }
    public double  getLength() { return mLength; }

    // Index is on [0, 1]
    public Vector2 interpolate(double index)
    {
    	Vector2 start = mStart.cpy();	// local copy
    	return start.lerp(mEnd, (float)index);
    }

    public double dotProduct(Vector2 _other)
    {
    	Vector2 other = _other.cpy();	// local copy
        Vector2 startToOther = other.sub(mStart);
        return mStartToEnd.dot(startToOther);
    }

    public ClosestPointReport getClosestPoint(Vector2 query_point) 
    {
        ClosestPointReport rv = new ClosestPointReport();
        if (mLength > kEpsilon) 
        {
            double dot_product = dotProduct(query_point);
            rv.index = dot_product / (mLength * mLength);
            rv.clamped_index = Math.min(1.0, Math.max(0.0, rv.index));
            rv.closest_point = interpolate(rv.index);
        } 
        else 
        {
            rv.index = rv.clamped_index = 0.0;
            rv.closest_point = new Vector2(mStart);
        }
        rv.distance = (query_point.sub(rv.closest_point)).len();
        return rv;
    }
}
