package org.team686.lib.util;

import java.util.Optional;

/*
 * A PathSegment consists of a start and end position
 * and various properties for the robot while on that segment
 * (the speed, lookahead distance, whether to use vision, etc.)
 */
public class PathSegment 
{
    protected static final double kEpsilon = 1E-9;

    protected Vector2d start;
    protected Vector2d end;
    protected Vector2d startToEnd; 		// pre-computed for efficiency
    protected double   length; 			// pre-computed for efficiency

    protected PathSegmentOptions options;
    
    static public class PathSegmentOptions
    {
	    protected double   speed;			// speed along this segment 
	    protected double   lookaheadDist;	// lookahead distance along this segment (use smaller for tighter turns) 
	    protected boolean  visionEnable;	// whether vision is enabled for this segment
	    									// vision will take over from path following when the target is identified
        protected Optional<String> marker;

	    // constructor
	    public PathSegmentOptions(double _speed, double _lookaheadDist, boolean _visionEnable)
	    {
	        speed = _speed;
	        lookaheadDist = _lookaheadDist;
	        visionEnable = _visionEnable;
          	marker = Optional.empty();
	    }

	    public PathSegmentOptions(double _speed, double _lookaheadDist, boolean _visionEnable, String _marker)
	    {
	        speed = _speed;
	        lookaheadDist = _lookaheadDist;
	        visionEnable = _visionEnable;
          	marker = Optional.of(_marker);
	    }

	    // copy constructor
	    public PathSegmentOptions(PathSegmentOptions _options)
	    {
	    	this(_options.speed, _options.lookaheadDist, _options.visionEnable);
	    }
	    
	    public double   getSpeed()   		{ return speed; }
	    public double   getLookaheadDist()  { return lookaheadDist; }
	    public boolean  getVisionEnable()   { return visionEnable; }
	    public Optional<String> getMarker() { return marker; }
    };
	    
	    
    public PathSegment(Vector2d _start, Vector2d _end, PathSegmentOptions _options) 
    {
        end = _end;
        updateStart(_start);
        options = new PathSegmentOptions(_options);
    }

    public void updateStart(Vector2d newStart)
    {
        start = newStart;		
        startToEnd = end.sub(start);
        length = startToEnd.length();
    }

    // copy constructor
    public PathSegment(PathSegment _seg)
    {
    	start 		= new Vector2d(_seg.start);
    	end   	   	= new Vector2d(_seg.end);
    	startToEnd 	= new Vector2d(_seg.startToEnd);
    	length		= _seg.length;
    	options		= new PathSegmentOptions(_seg.options);
    }
    
    public Vector2d getStart()   { return new Vector2d(start); }
    public Vector2d getEnd()     { return new Vector2d(end); }
    public double   getLength()  { return length; }
    public PathSegmentOptions getOptions() { return new PathSegmentOptions(options); }
    
    public Vector2d interpolate(double index)
    {
    	return start.interpolate(end, index);
    }

    // check alignment of vector (start, _that) to segment (start, end)
    public double dotProduct(Vector2d _that)
    {
        Vector2d startToThat = _that.sub(start);
        return startToEnd.dot(startToThat);
    }

    
    
    public static class ClosestPointReport 
    {
        public double index; 			// Index of the point on the path segment (not clamped to [0, 1])
        public double clamped_index; 	// As above, but clamped to [0, 1]
        public Vector2d closest_point; 	// The result of interpolate(clamped_index)
        public double distance; 		// The distance from closest_point to the query point
    }
    
    public ClosestPointReport getClosestPoint(Vector2d query_point) 
    {
        ClosestPointReport rv = new ClosestPointReport();
        if (length > kEpsilon) 
        {
            double dot_product = dotProduct(query_point);		// SQ = vector (start,query_point).  SE = vector(start, end)
            rv.index = dot_product / (length * length);			// index = |SQ|/|SE| cos(angle beteeen SQ & SE)   
            rv.clamped_index = Util.limit(rv.index, 0.0, 1.0);	// clamp in case |SQ| > |SE|
            rv.closest_point = interpolate(rv.index);			// point on SE closest to Q
        } 
        else 
        {
        	// segment is very small.  return start (which is near end)
            rv.index = rv.clamped_index = 0.0;
            rv.closest_point = new Vector2d(start);
        }
        rv.distance = query_point.distance(rv.closest_point);
        return rv;
    }
}
