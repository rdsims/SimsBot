package org.team686.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;
import org.mini2Dx.gdx.math.MathUtils;
import org.mini2Dx.gdx.math.Vector2;
import org.team686.lib.util.Pose;
import org.team686.lib.util.RigidTransform;

public class TestRigidTransform2d {
    public static final double kTestEpsilon = 1E-6;

/*    
    @Test
    public void testRotation2d() {
        // Test constructors
        Rotation2d rot1 = new Rotation2d();
        assertEquals(1, rot1.cos(), kTestEpsilon);
        assertEquals(0, rot1.sin(), kTestEpsilon);
        assertEquals(0, rot1.tan(), kTestEpsilon);
        assertEquals(0, rot1.getDegrees(), kTestEpsilon);
        assertEquals(0, rot1.getRadians(), kTestEpsilon);

        rot1 = new Rotation2d(1, 1, true);
        assertEquals(Math.sqrt(2) / 2, rot1.cos(), kTestEpsilon);
        assertEquals(Math.sqrt(2) / 2, rot1.sin(), kTestEpsilon);
        assertEquals(1, rot1.tan(), kTestEpsilon);
        assertEquals(45, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 4, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromRadians(Math.PI / 2);
        assertEquals(0, rot1.cos(), kTestEpsilon);
        assertEquals(1, rot1.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot1.tan());
        assertEquals(90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(270);
        assertEquals(0, rot1.cos(), kTestEpsilon);
        assertEquals(-1, rot1.sin(), kTestEpsilon);
        System.out.println(rot1.tan());
        assertTrue(-1 / kTestEpsilon > rot1.tan());
        assertEquals(-90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(-Math.PI / 2, rot1.getRadians(), kTestEpsilon);

        // Test inversion
        rot1 = Rotation2d.fromDegrees(270);
        Rotation2d rot2 = rot1.inverse();
        assertEquals(0, rot2.cos(), kTestEpsilon);
        assertEquals(1, rot2.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot2.tan());
        assertEquals(90, rot2.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot2.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(1);
        rot2 = rot1.inverse();
        assertEquals(rot1.cos(), rot2.cos(), kTestEpsilon);
        assertEquals(-rot1.sin(), rot2.sin(), kTestEpsilon);
        assertEquals(-1, rot2.getDegrees(), kTestEpsilon);

        // Test rotateBy
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        Rotation2d rot3 = rot1.rotateBy(rot2);
        assertEquals(0, rot3.cos(), kTestEpsilon);
        assertEquals(1, rot3.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot3.tan());
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot3.getRadians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.rotateBy(rot2);
        assertEquals(1, rot3.cos(), kTestEpsilon);
        assertEquals(0, rot3.sin(), kTestEpsilon);
        assertEquals(0, rot3.tan(), kTestEpsilon);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);
        assertEquals(0, rot3.getRadians(), kTestEpsilon);

        // A rotation times its inverse should be the identity
        Rotation2d identity = new Rotation2d();
        rot1 = Rotation2d.fromDegrees(21.45);
        rot2 = rot1.rotateBy(rot1.inverse());
        assertEquals(identity.cos(), rot2.cos(), kTestEpsilon);
        assertEquals(identity.sin(), rot2.sin(), kTestEpsilon);
        assertEquals(identity.getDegrees(), rot2.getDegrees(), kTestEpsilon);

        // Test interpolation
        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(90, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(135);
        rot3 = rot1.interpolate(rot2, .75);
        assertEquals(112.5, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(45);
        rot3 = rot1.interpolate(rot2, .5);
        assertEquals(45, rot3.getDegrees(), kTestEpsilon);
    }
*/
    
    @Test
    public void testTranslation2d() {
        // Test constructors
        Vector2 pos1 = new Vector2();
        assertEquals(0, pos1.x, kTestEpsilon);
        assertEquals(0, pos1.y, kTestEpsilon);
        assertEquals(0, pos1.len(), kTestEpsilon);

        pos1.x = 3;
        pos1.y = 4;
        assertEquals(3, pos1.x, kTestEpsilon);
        assertEquals(4, pos1.y, kTestEpsilon);
        assertEquals(5, pos1.len(), kTestEpsilon);

        pos1 = new Vector2(3, 4);
        assertEquals(3, pos1.x, kTestEpsilon);
        assertEquals(4, pos1.y, kTestEpsilon);
        assertEquals(5, pos1.len(), kTestEpsilon);

        // Test inversion
        pos1 = new Vector2(3.152f, 4.1666f);
        Vector2 pos2 = new Vector2(0,0).sub(pos1);
        assertEquals(-pos1.x, pos2.x, kTestEpsilon);
        assertEquals(-pos1.y, pos2.y, kTestEpsilon);
        assertEquals(pos1.len(), pos2.len(), kTestEpsilon);

        // Test rotateBy
        pos1 = new Vector2(2, 0);
        pos2 = new Vector2(pos1).rotate(90);
        assertEquals(0, pos2.x, kTestEpsilon);
        assertEquals(2, pos2.y, kTestEpsilon);
        assertEquals(pos1.len(), pos2.len(), kTestEpsilon);

        pos1 = new Vector2(2, 0);
        pos2 = new Vector2(pos1).rotate(-45);
        assertEquals(Math.sqrt(2), pos2.x, kTestEpsilon);
        assertEquals(-Math.sqrt(2), pos2.y, kTestEpsilon);
        assertEquals(pos1.len(), pos2.len(), kTestEpsilon);

        // Test translateBy
        pos1 = new Vector2(2, 0);
        pos2 = new Vector2(-2, 1);
        Vector2 pos3 = new Vector2(pos1).add(pos2);
        assertEquals(0, pos3.x, kTestEpsilon);
        assertEquals(1, pos3.y, kTestEpsilon);
        assertEquals(1, pos3.len(), kTestEpsilon);

        // A translation times its inverse should be the identity
        Vector2 identity = new Vector2();
        pos2 = pos1.add(new Vector2(0,0).sub(pos1));
        assertEquals(identity.x, pos2.x, kTestEpsilon);
        assertEquals(identity.y, pos2.y, kTestEpsilon);
        assertEquals(identity.len(), pos2.len(), kTestEpsilon);

        // Test interpolation
        pos1 = new Vector2(0, 1);
        pos2 = new Vector2(10, -1);
        pos3 = new Vector2(pos1).lerp(pos2, .5f);
        assertEquals(5, pos3.x, kTestEpsilon);
        assertEquals(0, pos3.y, kTestEpsilon);

        pos1 = new Vector2(0, 1);
        pos2 = new Vector2(10, -1);
        pos3 = new Vector2(pos1).lerp(pos2, .75f);
        assertEquals(7.5, pos3.x, kTestEpsilon);
        assertEquals(-.5, pos3.y, kTestEpsilon);
    }

    @Test
    public void testRigidTransform2d() {
        // Test constructors
        Pose pose1 = new Pose();
        assertEquals(0, pose1.getX(), kTestEpsilon);
        assertEquals(0, pose1.getY(), kTestEpsilon);
        assertEquals(0, pose1.getHeadingDeg(), kTestEpsilon);

        pose1 = new Pose(new Vector2(3, 4), 45*MathUtils.degreesToRadians);
        assertEquals(3, pose1.getX(), kTestEpsilon);
        assertEquals(4, pose1.getY(), kTestEpsilon);
        assertEquals(45, pose1.getHeadingDeg(), kTestEpsilon);

        // Test transformation
        pose1 = new Pose(new Vector2(3, 4), 90*MathUtils.degreesToRadians);
        RigidTransform T = new RigidTransform(new Vector2(1, 0), 0);
        Pose pose3 = pose1.transformBy(T);
        assertEquals(3, pose3.getX(), kTestEpsilon);
        assertEquals(5, pose3.getY(), kTestEpsilon);
        assertEquals(90, pose3.getHeadingDeg(), kTestEpsilon);

        pose1 = new Pose(new Vector2(3, 4),  90*MathUtils.degreesToRadians);
        T = new RigidTransform(new Vector2(1, 0), -90*MathUtils.degreesToRadians);
        pose3 = pose1.transformBy(T);
        assertEquals(3, pose3.getX(), kTestEpsilon);
        assertEquals(5, pose3.getY(), kTestEpsilon);
        assertEquals(0, pose3.getHeadingDeg(), kTestEpsilon);

        // A pose times its inverse should be the identity
        RigidTransform identity = new RigidTransform();
        RigidTransform T1 = new RigidTransform(new Vector2(3.51512152f, 4.23f), 91.6*MathUtils.degreesToRadians);
        RigidTransform T2 = T1.transformBy(T1.inverse());
        assertEquals(identity.getX(), T2.getX(), kTestEpsilon);
        assertEquals(identity.getY(), T2.getY(), kTestEpsilon);
        assertEquals(identity.getRotationDeg(), T2.getRotationDeg(), kTestEpsilon);

        // Test interpolation
        pose1 = new Pose(new Vector2(3, 4), 90*MathUtils.degreesToRadians);
        Pose pose2 = new Pose(new Vector2(13, -6), -90*MathUtils.degreesToRadians);
        pose3 = pose1.interpolate(pose2, .5);
        assertEquals(8, pose3.getX(), kTestEpsilon);
        assertEquals(-1, pose3.getY(), kTestEpsilon);
        assertEquals(0, pose3.getHeadingDeg(), kTestEpsilon);

        pose1 = new Pose(new Vector2(3, 4), 90*MathUtils.degreesToRadians);
        pose2 = new Pose(new Vector2(13, -6), -90*MathUtils.degreesToRadians);
        pose3 = pose1.interpolate(pose2, .75);
        assertEquals(10.5, pose3.getX(), kTestEpsilon);
        assertEquals(-3.5, pose3.getY(), kTestEpsilon);
        assertEquals(-45, pose3.getHeadingDeg(), kTestEpsilon);
    }
}
