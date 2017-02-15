package org.team686.lib.util.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.team686.lib.util.*;
import org.team686.lib.util.Path.Waypoint;

import org.junit.Test;
import org.mini2Dx.gdx.math.Vector2;

public class PathTest {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testPathSegment() {
        Vector2 start = new Vector2(0, 0);
        Vector2 end = new Vector2(1, 0);
        PathSegment segment = new PathSegment(start, end, 1);
        assertEquals(1, segment.getLength(), kTestEpsilon);
        assertEquals(start.x, segment.getStart().x, kTestEpsilon);
        assertEquals(start.y, segment.getStart().y, kTestEpsilon);
        assertEquals(end.x, segment.getEnd().x, kTestEpsilon);
        assertEquals(end.y, segment.getEnd().y, kTestEpsilon);

        // Update start
        start = new Vector2(0.5f, 0);
        segment.updateStart(start);
        assertEquals(0.5, segment.getLength(), kTestEpsilon);
        assertEquals(start.x, segment.getStart().x, kTestEpsilon);
        assertEquals(start.y, segment.getStart().y, kTestEpsilon);

        // Interpolate
        Vector2 midpoint = segment.interpolate(0.5);
        assertEquals(.75, midpoint.x, kTestEpsilon);
        assertEquals(0, midpoint.y, kTestEpsilon);

        // GetClosestPoint - point on path
        PathSegment.ClosestPointReport report = segment.getClosestPoint(midpoint);
        assertEquals(.5, report.index, kTestEpsilon);
        assertEquals(.5, report.clamped_index, kTestEpsilon);
        assertEquals(midpoint.x, report.closest_point.x, kTestEpsilon);
        assertEquals(midpoint.y, report.closest_point.y, kTestEpsilon);
        assertEquals(0, report.distance, kTestEpsilon);

        // GetClosestPoint - point off of path
        report = segment.getClosestPoint(new Vector2(.75f, 1));
        assertEquals(.5, report.index, kTestEpsilon);
        assertEquals(.5, report.clamped_index, kTestEpsilon);
        assertEquals(midpoint.x, report.closest_point.x, kTestEpsilon);
        assertEquals(midpoint.y, report.closest_point.y, kTestEpsilon);
        assertEquals(1, report.distance, kTestEpsilon);

        // GetClosestPoint - point behind start
        report = segment.getClosestPoint(new Vector2(0, 1));
        assertEquals(-1, report.index, kTestEpsilon);
        assertEquals(0, report.clamped_index, kTestEpsilon);
        assertEquals(start.x, report.closest_point.x, kTestEpsilon);
        assertEquals(start.y, report.closest_point.y, kTestEpsilon);
        assertEquals(Math.hypot(.5, 1), report.distance, kTestEpsilon);

        // GetClosestPoint - point after end
        report = segment.getClosestPoint(new Vector2(2, -1));
        assertEquals(3, report.index, kTestEpsilon);
        assertEquals(1, report.clamped_index, kTestEpsilon);
        assertEquals(end.x, report.closest_point.x, kTestEpsilon);
        assertEquals(end.y, report.closest_point.y, kTestEpsilon);
        assertEquals(Math.hypot(1, 1), report.distance, kTestEpsilon);
    }

    @Test
    public void testPath() {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Vector2(0, 0), 1));
        waypoints.add(new Waypoint(new Vector2(1, 0), 1));
        waypoints.add(new Waypoint(new Vector2(2, 0), 1));
        waypoints.add(new Waypoint(new Vector2(2, 1), 1));
        waypoints.add(new Waypoint(new Vector2(2, 2), 1));

        Path path = new Path(waypoints);
        assertEquals(4, path.getRemainingLength(), kTestEpsilon);

        Vector2 robot_position = new Vector2(0, 0);
        double distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(4, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(.5f, 0);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(3.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(1, 0);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(3, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(1, .5f);
        distance = path.update(robot_position);
        assertEquals(.5, distance, kTestEpsilon);
        assertEquals(3, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(2.5f, .5f);
        distance = path.update(robot_position);
        assertEquals(.5, distance, kTestEpsilon);
        assertEquals(1.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(0, 0);
        distance = path.update(robot_position);
        assertTrue(distance > 1);
        assertEquals(1.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(2.5f, 2.5f);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(0, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Vector2(0, 0);
        distance = path.update(robot_position);
        assertEquals(0, distance, kTestEpsilon);
        assertEquals(0, path.getRemainingLength(), kTestEpsilon);
    }

    @Test
    public void testLookahead() {
        List<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Vector2(0, 0), 1));
        waypoints.add(new Waypoint(new Vector2(1, 0), 1));
        waypoints.add(new Waypoint(new Vector2(2, 0), 1));
        waypoints.add(new Waypoint(new Vector2(2, 1), 1));
        waypoints.add(new Waypoint(new Vector2(2, 2), 1));
        Path path = new Path(waypoints);

        // Robot at path start, lookahead 1 unit
        Vector2 robot_position = new Vector2(0, 0);
        path.update(robot_position);
        PathSegment.Sample lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertEquals(1, lookahead_point.position.x, kTestEpsilon);
        assertEquals(0, lookahead_point.position.y, kTestEpsilon);

        // Robot at path start, lookahead 2 units
        robot_position = new Vector2(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 2);
        assertEquals(2, lookahead_point.position.x, kTestEpsilon);
        assertEquals(0, lookahead_point.position.y, kTestEpsilon);

        // Robot at path start, lookahead 2.1 units
        robot_position = new Vector2(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 2.1);
        assertEquals(2, lookahead_point.position.x, kTestEpsilon);
        assertTrue(0 < lookahead_point.position.y);

        // Robot near path start, lookahead 1 unit
        robot_position = new Vector2(0, 0.1f);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertTrue(1 > lookahead_point.position.x);
        assertEquals(0, lookahead_point.position.y, kTestEpsilon);

        // Robot behind path start, lookahead 1 unit
        robot_position = new Vector2(-.5f, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertEquals(.5, lookahead_point.position.x, kTestEpsilon);
        assertEquals(0, lookahead_point.position.y, kTestEpsilon);

        // Lookahead goes past end
        robot_position = new Vector2(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 5);
        assertEquals(2, lookahead_point.position.x, kTestEpsilon);
        assertTrue(2 < lookahead_point.position.y);
    }

    @Test
    public void testNumericalStability() {
        Random rand = new Random(1);
        for (int i = 0; i < 10000; ++i) {
            List<Waypoint> waypoints = new ArrayList<>();
            waypoints.add(new Waypoint(new Vector2(18, 26), 120.0));
            waypoints.add(new Waypoint(new Vector2(24, 18), 120.0));
            waypoints.add(new Waypoint(new Vector2(90, 18), 120.0, "PopHood"));
            waypoints.add(new Waypoint(new Vector2(205, 18), 120.0));
            Path path = new Path(waypoints);
            for (int j = 0; j < 50; ++j) {
                Vector2 robot_position = new Vector2((float)(rand.nextDouble() * 10.0 + 24),
                        (float)(rand.nextDouble() * 10.0 + 18));
                path.update(robot_position);
                assertTrue(path.getMarkersCrossed().isEmpty());
            }
        }
    }
}
