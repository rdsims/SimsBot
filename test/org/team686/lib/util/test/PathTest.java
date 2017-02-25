package org.team686.lib.util.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.team686.lib.util.*;
import org.team686.lib.util.Path.Waypoint;

import org.junit.Test;

public class PathTest {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testPathSegment() 
    {
    	double maxSpeed = 1;
    	double maxAccel = 1;
    	double lookaheadDist = 1;
    	boolean visionEnable = false;
    	
    	PathSegment.Options options = new PathSegment.Options(maxSpeed, maxAccel, lookaheadDist, visionEnable);
    	
        Vector2d start = new Vector2d(0, 0);
        Vector2d end = new Vector2d(1, 0);
        PathSegment segment = new PathSegment(start, end, options);
        assertEquals(1, segment.getLength(), kTestEpsilon);
        assertEquals(start.getX(), segment.getStart().getX(), kTestEpsilon);
        assertEquals(start.getY(), segment.getStart().getY(), kTestEpsilon);
        assertEquals(end.getX(), segment.getEnd().getX(), kTestEpsilon);
        assertEquals(end.getY(), segment.getEnd().getY(), kTestEpsilon);

        // Update start
        start = new Vector2d(0.5, 0);
        segment.updateStart(start);
        assertEquals(0.5, segment.getLength(), kTestEpsilon);
        assertEquals(start.getX(), segment.getStart().getX(), kTestEpsilon);
        assertEquals(start.getY(), segment.getStart().getY(), kTestEpsilon);

        // Interpolate
        Vector2d midpoint = segment.interpolate(0.5);
        assertEquals(.75, midpoint.getX(), kTestEpsilon);
        assertEquals(0, midpoint.getY(), kTestEpsilon);

        // GetClosestPoint - point on path
        Util.ClosestPointOnSegment report = segment.getClosestPoint(midpoint);
        assertEquals(.5, report.index, kTestEpsilon);
        assertEquals(.5, report.clampedIndex, kTestEpsilon);
        assertEquals(midpoint.getX(), report.point.getX(), kTestEpsilon);
        assertEquals(midpoint.getY(), report.point.getY(), kTestEpsilon);
        assertEquals(0, report.distance, kTestEpsilon);

        // GetClosestPoint - point off of path
        report = segment.getClosestPoint(new Vector2d(.75, 1));
        assertEquals(.5, report.index, kTestEpsilon);
        assertEquals(.5, report.clampedIndex, kTestEpsilon);
        assertEquals(midpoint.getX(), report.point.getX(), kTestEpsilon);
        assertEquals(midpoint.getY(), report.point.getY(), kTestEpsilon);
        assertEquals(1, report.distance, kTestEpsilon);

        // GetClosestPoint - point behind start
        report = segment.getClosestPoint(new Vector2d(0, 1));
        assertEquals(-1, report.index, kTestEpsilon);
        assertEquals(0, report.clampedIndex, kTestEpsilon);
        assertEquals(start.getX(), report.point.getX(), kTestEpsilon);
        assertEquals(start.getY(), report.point.getY(), kTestEpsilon);
        assertEquals(Math.hypot(.5, 1), report.distance, kTestEpsilon);

        // GetClosestPoint - point after end
        report = segment.getClosestPoint(new Vector2d(2, -1));
        assertEquals(3, report.index, kTestEpsilon);
        assertEquals(1, report.clampedIndex, kTestEpsilon);
        assertEquals(end.getX(), report.point.getX(), kTestEpsilon);
        assertEquals(end.getY(), report.point.getY(), kTestEpsilon);
        assertEquals(Math.hypot(1, 1), report.distance, kTestEpsilon);
    }

    @Test
    public void testPath() 
    {
    	boolean reversed = false;
    	
    	double maxSpeed = 1;
    	double maxAccel = 1;
    	double lookaheadDist = 1;
    	boolean visionEnable = false;
    	
    	PathSegment.Options options = new PathSegment.Options(maxSpeed, maxAccel, lookaheadDist, visionEnable);

    	List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Vector2d(0, 0), options));
        waypoints.add(new Waypoint(new Vector2d(1, 0), options));
        waypoints.add(new Waypoint(new Vector2d(2, 0), options));
        waypoints.add(new Waypoint(new Vector2d(2, 1), options));
        waypoints.add(new Waypoint(new Vector2d(2, 2), options));

        Path path = new Path(waypoints, reversed);
        assertEquals(4, path.getRemainingLength(), kTestEpsilon);

        Vector2d robot_position = new Vector2d(0, 0);
        double distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(4, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(.5f, 0);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(3.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(1, 0);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(3, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(1, .5);
        distance = path.update(robot_position);
        assertEquals(.5, distance, kTestEpsilon);
        assertEquals(3, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(2.5, .5);
        distance = path.update(robot_position);
        assertEquals(.5, distance, kTestEpsilon);
        assertEquals(1.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(0, 0);
        distance = path.update(robot_position);
        assertTrue(distance > 1);
        assertEquals(1.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(2.5, 2.5);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(0, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2d(0, 0);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(0, path.getRemainingLength(), kTestEpsilon);
    }

    @Test
    public void testLookahead() 
    {
    	boolean reversed = false;
    	
    	double maxSpeed = 1;
    	double maxAccel = 1;
    	double lookaheadDist = 1;
    	boolean visionEnable = false;
    	
    	PathSegment.Options options = new PathSegment.Options(maxSpeed, maxAccel, lookaheadDist, visionEnable);

    	List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Vector2d(0, 0), options));
        waypoints.add(new Waypoint(new Vector2d(1, 0), options));
        waypoints.add(new Waypoint(new Vector2d(2, 0), options));
        waypoints.add(new Waypoint(new Vector2d(2, 1), options));
        waypoints.add(new Waypoint(new Vector2d(2, 2), options));
        Path path = new Path(waypoints, reversed);

        // Robot at path start, lookahead 1 unit
        Vector2d robot_position = new Vector2d(0, 0);
        path.update(robot_position);
        Vector2d lookahead_point = path.getLookaheadPoint(robot_position, 0);
        assertEquals(1, lookahead_point.getX(), kTestEpsilon);
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Robot at path start, lookahead 2 units
        robot_position = new Vector2d(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertEquals(2, lookahead_point.getX(), kTestEpsilon);
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Robot at path start, lookahead 2.1 units
        robot_position = new Vector2d(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 1.1);
        assertEquals(2, lookahead_point.getX(), kTestEpsilon);
        assertTrue(0 < lookahead_point.getY());

        // Robot near path start, lookahead 1 unit
        robot_position = new Vector2d(0, 0.1);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 0);
        assertTrue(1 > lookahead_point.getX());
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Robot behind path start, lookahead 1 unit
        robot_position = new Vector2d(-.5, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 0);
        assertEquals(.5, lookahead_point.getX(), kTestEpsilon);
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Lookahead goes past end
        robot_position = new Vector2d(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 4);
        assertEquals(2, lookahead_point.getX(), kTestEpsilon);
        assertTrue(2 < lookahead_point.getY());
    }

    // TODO: add simulation test, including reversed path
    
    @Test
    public void testNumericalStability() 
    {
    	boolean reversed = false;
    	
    	double maxSpeed = 120.0;
    	double maxAccel = 90.0;
    	double lookaheadDist = 1;
    	boolean visionEnable = false;
    	
    	PathSegment.Options options = new PathSegment.Options(maxSpeed, maxAccel, lookaheadDist, visionEnable);
    	
        Random rand = new Random(1);
        for (int i = 0; i < 10000; ++i) 
        {
            List<Waypoint> waypoints = new ArrayList<>();
            waypoints.add(new Waypoint(new Vector2d( 18, 26), options));
            waypoints.add(new Waypoint(new Vector2d( 24, 18), options));
            waypoints.add(new Waypoint(new Vector2d( 90, 18), options));
            waypoints.add(new Waypoint(new Vector2d(205, 18), options));
            Path path = new Path(waypoints, reversed);
            for (int j = 0; j < 50; ++j) 
            {
                Vector2d robot_position = new Vector2d(rand.nextDouble() * 10.0 + 24, rand.nextDouble() * 10.0 + 18);
                path.update(robot_position);
                assertTrue(path.getMarkersCrossed().isEmpty());
            }
        }
    }
}
