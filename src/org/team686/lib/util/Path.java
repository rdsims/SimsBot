package org.team686.lib.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.team686.lib.util.PathSegment.PathSegmentOptions;

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

    protected List<Waypoint> waypoints;
    protected List<PathSegment> segments;
    protected Set<String> markersCrossed;

    /**
     * A point along the Path, which consists of the location, the speed, and a
     * string marker (that future code can identify). 
     * Paths consist of a List of Waypoints.
     */
    public static class Waypoint 
    {
        public final Vector2d position;
        public final PathSegmentOptions options;

        public Waypoint(Vector2d _position, PathSegmentOptions _options) 
        {
            position = _position;
            options  = _options;
        }
    }

    
    // construct a path given a list of waypoints
    public Path(List<Waypoint> _waypoints) 
    {
    	// note to calling function: first waypoint should be near actual starting point (but doesn't have to be) 
    	
        waypoints = new ArrayList<Waypoint>(_waypoints);	// make a copy of waypoint list
        
        // construct path segments from each pair of waypoints
        segments = new ArrayList<PathSegment>();	
        for (int i = 0; i < _waypoints.size() - 1; ++i) 
        {
            segments.add( new PathSegment(waypoints.get(i).position, waypoints.get(i+1).position, waypoints.get(i).options ));
        }
        
        markersCrossed = new HashSet<String>();
    }


    public PathSegment getCurrentSegment() { return new PathSegment(segments.get(0)); }

    /*
     *  update() takes the current robot position, and updates the progress along the path
     *  removing waypoints already passed and adding markersCrossed
     *  
     *  update() returns the distance off of the path
     */
    public double update(Vector2d _position) 
    {
        double distOffPath = 0.0;
        
        for (Iterator<PathSegment> it = segments.iterator(); it.hasNext();) 
        {
        	// calculate distance from segment
            PathSegment currSegment = it.next();
            PathSegment.ClosestPointReport closestPointReport = currSegment.getClosestPoint(_position);
            
            // check if segment has been completed
            if (closestPointReport.index >= kSegmentCompletePercentage) 
            {
                // segment complete: mark as crossed, remove segment and waypoint from beginning of list
            	markerCrossed(currSegment);
                it.remove();
                waypoints.remove(0);
            } 
            else 
            {
            	// segment not complete: move segment start to closest point 
                if (closestPointReport.index > 0.0) 
                {
                    // Can shorten this segment
                    currSegment.updateStart(closestPointReport.closest_point);
                }
                
                distOffPath = closestPointReport.distance;
                
                
                // check if next segment is closer than this one
                if (it.hasNext()) 
                {
                    PathSegment nextSegment = it.next();
                    PathSegment.ClosestPointReport nextSegClosestPointReport = nextSegment.getClosestPoint(_position);
                    
                    if (nextSegClosestPointReport.index > 0 &&
                        nextSegClosestPointReport.index < kSegmentCompletePercentage &&
                        nextSegClosestPointReport.distance < distOffPath) 
                    {
                    	// next segment is closer: drop current segment and move to next
                        nextSegment.updateStart(nextSegClosestPointReport.closest_point);
                        distOffPath = nextSegClosestPointReport.distance;
                        
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
    public Vector2d getLookaheadPoint(Vector2d _position, double _lookaheadDistance) 
    {
        if (segments.size() == 0) 
        {
        	// already finished path.  this shouldn't happen.
            return new Vector2d();
        }

        // Check the distances to the start and end of each segment. As soon as
        // we find a point > lookahead_distance away, we know the right point
        // lies somewhere on that segment.
        Vector2d start = segments.get(0).getStart();
        double distanceToStart = start.distance(_position);
        if (distanceToStart >= _lookaheadDistance) 
        {
        	// Special case: 
            // not within range of start, so first attempt to to get back to start 
            return start;
        }
        
        // find first segment whose endpoint is outside of lookahead circle
        for (int i = 0; i < segments.size(); ++i) 
        {
            PathSegment segment = segments.get(i);
            double distance = segment.getEnd().distance(_position);
            if (distance >= _lookaheadDistance) 
            {
                // This segment contains the lookahead point
                Optional<Vector2d> intersectionPoint = getPathLookaheadCircleIntersection(segment, _position, _lookaheadDistance);
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
        double interpFactor = 2*_lookaheadDistance / lastSegment.getLength();
        PathSegment extrapolatedLastSegment = new PathSegment(lastSegment.getStart(), lastSegment.interpolate(interpFactor), lastSegment.options);
        Optional<Vector2d> intersectionPoint = getPathLookaheadCircleIntersection(extrapolatedLastSegment, _position, _lookaheadDistance);
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
