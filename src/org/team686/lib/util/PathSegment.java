package org.team686.lib.util;


/**
 * A PathSegment consists of two Vector objects (the start and end
 * points) as well as the speed of the robot.
 *
 */
public class PathSegment {
    protected static final double kEpsilon = 1E-9;

    public static class Sample
    {
        public final Vector2d position;
        public final double speed;

        public Sample(Vector2d _position, double _speed) 
        {
            this.position = _position;
            this.speed = _speed;
        }
    }

    protected double mSpeed;
    protected Vector2d mStart;
    protected Vector2d mEnd;
    protected Vector2d mStartToEnd; // pre-computed for efficiency
    protected double mLength; // pre-computed for efficiency

    public static class ClosestPointReport 
    {
        public double index; // Index of the point on the path segment (not
                             // clamped to [0, 1])
        public double clamped_index; // As above, but clamped to [0, 1]
        public Vector2d closest_point; // The result of
                                            // interpolate(clamped_index)
        public double distance; // The distance from closest_point to the query
                                // point
    }

    public PathSegment(Vector2d start, Vector2d end, double speed) 
    {
        mEnd = end;
        mSpeed = speed;
        updateStart(start);
    }

    public void updateStart(Vector2d new_start)
    {
        mStart = new_start;		
        mStartToEnd = mEnd.sub(mStart);
        mLength = mStartToEnd.length();
    }

    public double getSpeed()  { return mSpeed; }
    public Vector2d getStart()  { return mStart; }
    public Vector2d getEnd()    { return mEnd; }
    public double getLength() { return mLength; }

    // Index is on [0, 1]
    public Vector2d interpolate(double index)
    {
    	return mStart.interpolate(mEnd, index);
    }

    public double dotProduct(Vector2d _other)
    {
        Vector2d startToOther = _other.sub(mStart);
        return mStartToEnd.dot(startToOther);
    }

    public ClosestPointReport getClosestPoint(Vector2d query_point) 
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
            rv.closest_point = new Vector2d(mStart);
        }
        rv.distance = query_point.distance(rv.closest_point);
        return rv;
    }
}
