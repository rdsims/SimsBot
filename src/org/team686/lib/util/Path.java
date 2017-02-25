package org.team686.lib.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.team686.lib.util.PathSegment.Options;

/**
 * A Path is a recording of the path that the robot takes. Path objects consist
 * of a List of Waypoints that the robot passes by. Using multiple Waypoints in
 * a Path object and the robot's current speed, the code can extrapolate future
 * Waypoints and predict the robot's motion. It can also dictate the robot's
 * motion along the set path.
 */
public class Path 
{
    protected static final double kSegmentCompletePercentage = .99;

    protected boolean reversed;			
    protected List<Waypoint> waypoints;
    protected List<PathSegment> segments;
    protected Set<String> markersCrossed;
    private double lookaheadDistance;

    // when path is reversed:
    // 1. start/end of each segment are swapped for the purposes of keeping track of progress
    // 2. robot is virtually rotated 180 degrees for the purpose of finding lookahead point
    // 3. wheel velocities are negated (so that the robot drives backwards)
    
    /**
     * A point along the Path, which consists of the location, the speed, and a
     * string marker (that future code can identify). 
     * Paths consist of a List of Waypoints.
     */
    public static class Waypoint 
    {
        public final Vector2d position;
        public final Options options;

        public Waypoint(Vector2d _position, Options _options) 
        {
            position = _position;
            options  = _options;
        }
    }

    // constructor for empty path
    public Path()
    {
    	this(new ArrayList<Waypoint>(), false);
    }
    
    
    // construct a path given a list of waypoints
    public Path(List<Waypoint> _waypoints, boolean _reversed) 
    {
    	// note to calling function: first waypoint should be near actual starting point (but doesn't have to be) 
    	
        waypoints = new ArrayList<Waypoint>(_waypoints);	// make a copy of waypoint list
        reversed = _reversed;
        
        // construct path segments from each pair of waypoints
        segments = new ArrayList<PathSegment>();
        if (reversed)
        {
	        for (int i = _waypoints.size() - 1; i > 0 ; --i) 
	        {
	            segments.add( new PathSegment(waypoints.get(i).position, waypoints.get(i-1).position, waypoints.get(i-1).options ));
	        }
        }        
        else
        {
	        for (int i = 0; i < _waypoints.size() - 1; ++i) 
	        {
	            segments.add( new PathSegment(waypoints.get(i).position, waypoints.get(i+1).position, waypoints.get(i).options ));
	        }
        }        
        markersCrossed = new HashSet<String>();
    }


    public boolean getReversed() { return reversed; }
    
    public double getLookaheadDistance() { return lookaheadDistance; }	// return lookahead distance used by last call to getLookaheadPoint()
    
    public Optional<PathSegment> getCurrentSegment()
    { 
    	if (segments.isEmpty())
    		return Optional.empty();
		else
    		return Optional.of(new PathSegment(segments.get(0))); 
    }
    
    public double getSegmentMaxSpeed() 
    {
    	Optional<PathSegment> segment = getCurrentSegment();
    	if (segment.isPresent())
    		return segment.get().getOptions().getMaxSpeed();
    	else
    		return 0;
    }

    public double getSegmentMaxAccel()
    {
    	Optional<PathSegment> segment = getCurrentSegment();
    	if (segment.isPresent())
    		return segment.get().getOptions().getMaxAccel();
    	else
    		return 0;
    }

    public boolean getSegmentVisionEnable()
    {
    	Optional<PathSegment> segment = getCurrentSegment();
    	if (segment.isPresent())
    		return segment.get().getOptions().getVisionEnable();
    	else
    		return true;	// assume no segments defined when vision only
    }
    
    /*
     *  update() takes the current robot position, and updates the progress along the path
     *  removing waypoints already passed and adding markersCrossed
     *  
     *  update() returns the distance off of the path
     */
    public double update(Vector2d _position) 
    {
        double distOffPath = 0.0;
        Util.ClosestPointOnSegment closestPoint;
        
        for (Iterator<PathSegment> it = segments.iterator(); it.hasNext();) 
        {
        	// calculate distance from segment
            PathSegment currSegment = it.next();
            closestPoint = currSegment.getClosestPoint(_position);
            
            // check if segment has been completed
            if (closestPoint.index >= kSegmentCompletePercentage) 
            {
                // segment complete: mark as crossed, remove segment and waypoint from beginning of list
            	markerCrossed(currSegment);
                it.remove();
                waypoints.remove(0);
            } 
            else 
            {
            	// segment not complete: move segment start to closest point 
                if (closestPoint.index > 0.0) 
                {
                    // Can shorten this segment
                    currSegment.updateStart(closestPoint.point);
                }
                
                distOffPath = closestPoint.distance;
                
                
                // check if next segment is closer than this one
                if (it.hasNext()) 
                {
                    PathSegment nextSegment = it.next();
                    closestPoint = nextSegment.getClosestPoint(_position);
                    
                    if (closestPoint.index > 0 &&
                    	closestPoint.index < kSegmentCompletePercentage &&
                    	closestPoint.distance < distOffPath) 
                    {
                    	// next segment is closer: drop current segment and move to next
                        nextSegment.updateStart(closestPoint.point);
                        distOffPath = closestPoint.distance;
                        
                    	markerCrossed(currSegment);
                        segments.remove(0);
                        waypoints.remove(0);
                    }
                }
                
                // break out of loop once we've found a segment not yet completed 
                break;
            }
        }
        return distOffPath;
    }

    public void markerCrossed(PathSegment _segment)
    {
    	Optional<String> marker = _segment.getOptions().getMarker();
    	
    	if (marker.isPresent())
    	{
    		markersCrossed.add( marker.get() );
    	}
    }
    
    public Set<String> getMarkersCrossed() 
    {
        return markersCrossed;
    }

    public double getRemainingLength() 
    {
        double length = 0.0;
        for (int i = 0; i < segments.size(); ++i) 
            length += segments.get(i).getLength();

        return length;
    }

    
    // getLookaheadPoint() finds a point which is lookaheadDistance ahead along path from current position
    //   The lookahead point will be at the intersection of that path and 
    //   a circle centered at _position with radius _lookaheadDistance
    public Vector2d getLookaheadPoint(Vector2d _position, double _distanceFromPath) 
    {
        if (segments.size() == 0) 
        {
        	// already finished path.  this shouldn't happen.
            return new Vector2d();
        }

        // Check the distances to the start and end of each segment. As soon as
        // we find a point > lookahead_distance away, we know the right point
        // lies somewhere on that segment.
        PathSegment segment = segments.get(0);
        Vector2d start = segment.getStart();
        lookaheadDistance = segment.getOptions().getLookaheadDist() + _distanceFromPath;
        double distanceToStart = start.distance(_position);
        if (distanceToStart >= lookaheadDistance) 
        {
        	// Special case: 
            // not within range of start, so first attempt to to get back to start
            return start;
        }
        
        // find first segment whose endpoint is outside of lookahead circle
        for (int i = 0; i < segments.size(); ++i) 
        {
            segment = segments.get(i);
            double distanceToEnd = segment.getEnd().distance(_position);
            lookaheadDistance = segment.getOptions().getLookaheadDist() + _distanceFromPath;
            if (distanceToEnd <= lookaheadDistance) 
            {
                // This segment contains the lookahead point
                Optional<Vector2d> intersectionPoint = getPathLookaheadCircleIntersection(segment, _position, lookaheadDistance);
                if (intersectionPoint.isPresent()) 
                {
                    return new Vector2d(intersectionPoint.get());
                } 
                else 
                {
                	// shouldn't happen unless path is discontiguous
                	// Path() constructor always makes contiguous paths
                    System.out.println("ERROR: No intersection point?");
                }
            }
        }
        
        // Special case:
        // Last point has moved inside lookahead circle
        // Extrapolate last segment forward and return intersection with extrapolated segment
        PathSegment lastSegment = segments.get(segments.size() - 1);
        // calculate interpolation factor to guarantee intersection
        lookaheadDistance = lastSegment.getOptions().getLookaheadDist() + _distanceFromPath;
        double interpFactor = 2*lookaheadDistance / lastSegment.getLength();
        PathSegment extrapolatedLastSegment = new PathSegment(lastSegment.getStart(), lastSegment.interpolate(interpFactor), lastSegment.options);
        Optional<Vector2d> intersectionPoint = getPathLookaheadCircleIntersection(extrapolatedLastSegment, _position, lookaheadDistance);
        if (intersectionPoint.isPresent()) 
        {
            return new Vector2d(intersectionPoint.get());
        } 
        else 
        {
        	// shouldn't happen.  drive towards endpoint
            System.out.println("ERROR: No intersection point anywhere on line?");
            return new Vector2d(lastSegment.getEnd());
        }
    }

    static Optional<Vector2d> getPathLookaheadCircleIntersection(PathSegment _segment, Vector2d _center, double _radius)
    {
    	Optional<Vector2d[]> intersectionPoints = Util.lineCircleIntersection(_segment.start, _segment.end, _center, _radius);

    	if (intersectionPoints.isPresent())
    	{
    		Vector2d[] soln = intersectionPoints.get();
    		
    		if (soln.length == 1)
    			return Optional.of(soln[0]);
    		else
    		{
    			// 2 solutions returned.  Choose the one that is closest to end (largest positive dot product)
    			double dot0 = _segment.dotProduct(soln[0]);
    			double dot1 = _segment.dotProduct(soln[1]);
    			
    			if (dot0 >= dot1)			
    				return Optional.of(soln[0]);
    			else
    				return Optional.of(soln[1]);	
    		}
    	}
    	else
    	{
    		// no intersection
    		return Optional.empty();
    	}
    }
}
