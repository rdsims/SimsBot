package org.team686.lib.util.test;

import static org.junit.Assert.*;

import org.junit.Test;
import org.team686.lib.util.Pose;
import org.team686.lib.util.Vector2d;

public class TestRigidTransform2d 
{
    public static final double kTestEpsilon = 1E-9;

/*    
    @Test
    public void testRotation2d() {
        // Test constructors
        Rotation2d rot1 = new Rotation2d();
        assertEquals(1, rot1.cos(), kTestEpsilon);
        assertEquals(0, rot1.sin(), kTestEpsilon);
        assertEquals(0, rot1.tan(), kTestEpsilon);
        assertEquals(0, rot1.getDegrees(), kTestEpsilon);
        assertEquals(0, rot1.getians(), kTestEpsilon);

        rot1 = new Rotation2d(1, 1, true);
        assertEquals(Math.sqrt(2) / 2, rot1.cos(), kTestEpsilon);
        assertEquals(Math.sqrt(2) / 2, rot1.sin(), kTestEpsilon);
        assertEquals(1, rot1.tan(), kTestEpsilon);
        assertEquals(45, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 4, rot1.getians(), kTestEpsilon);

        rot1 = Rotation2d.fromians(Math.PI / 2);
        assertEquals(0, rot1.cos(), kTestEpsilon);
        assertEquals(1, rot1.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot1.tan());
        assertEquals(90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot1.getians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(270);
        assertEquals(0, rot1.cos(), kTestEpsilon);
        assertEquals(-1, rot1.sin(), kTestEpsilon);
        System.out.println(rot1.tan());
        assertTrue(-1 / kTestEpsilon > rot1.tan());
        assertEquals(-90, rot1.getDegrees(), kTestEpsilon);
        assertEquals(-Math.PI / 2, rot1.getians(), kTestEpsilon);

        // Test inversion
        rot1 = Rotation2d.fromDegrees(270);
        Rotation2d rot2 = rot1.inverse();
        assertEquals(0, rot2.cos(), kTestEpsilon);
        assertEquals(1, rot2.sin(), kTestEpsilon);
        assertTrue(1 / kTestEpsilon < rot2.tan());
        assertEquals(90, rot2.getDegrees(), kTestEpsilon);
        assertEquals(Math.PI / 2, rot2.getians(), kTestEpsilon);

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
        assertEquals(Math.PI / 2, rot3.getians(), kTestEpsilon);

        rot1 = Rotation2d.fromDegrees(45);
        rot2 = Rotation2d.fromDegrees(-45);
        rot3 = rot1.rotateBy(rot2);
        assertEquals(1, rot3.cos(), kTestEpsilon);
        assertEquals(0, rot3.sin(), kTestEpsilon);
        assertEquals(0, rot3.tan(), kTestEpsilon);
        assertEquals(0, rot3.getDegrees(), kTestEpsilon);
        assertEquals(0, rot3.getians(), kTestEpsilon);

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
        Vector2d pos1 = new Vector2d();
        assertEquals(0, pos1.getX(), kTestEpsilon);
        assertEquals(0, pos1.getY(), kTestEpsilon);
        assertEquals(0, pos1.length(), kTestEpsilon);

        pos1.setX(3);
        pos1.setY(4);
        assertEquals(3, pos1.getX(), kTestEpsilon);
        assertEquals(4, pos1.getY(), kTestEpsilon);
        assertEquals(5, pos1.length(), kTestEpsilon);

        pos1 = new Vector2d(3, 4);
        assertEquals(3, pos1.getX(), kTestEpsilon);
        assertEquals(4, pos1.getY(), kTestEpsilon);
        assertEquals(5, pos1.length(), kTestEpsilon);

        // Test inversion
        pos1 = new Vector2d(3.152f, 4.1666f);
        Vector2d pos2 = pos1.neg();
        assertEquals(-pos1.getX(), pos2.getX(), kTestEpsilon);
        assertEquals(-pos1.getY(), pos2.getY(), kTestEpsilon);
        assertEquals(pos1.length(), pos2.length(), kTestEpsilon);

        // Test rotateBy
        pos1 = new Vector2d(2, 0);
        pos2 = pos1.rotateDeg(90);
        assertEquals(0, pos2.getX(), kTestEpsilon);
        assertEquals(2, pos2.getY(), kTestEpsilon);
        assertEquals(pos1.length(), pos2.length(), kTestEpsilon);

        pos1 = new Vector2d(2, 0);
        pos2 = pos1.rotateDeg(-45);
        assertEquals(Math.sqrt(2), pos2.getX(), kTestEpsilon);
        assertEquals(-Math.sqrt(2), pos2.getY(), kTestEpsilon);
        assertEquals(pos1.length(), pos2.length(), kTestEpsilon);

        // Test translateBy
        pos1 = new Vector2d(2, 0);
        pos2 = new Vector2d(-2, 1);
        Vector2d pos3 = pos1.add(pos2);
        assertEquals(0, pos3.getX(), kTestEpsilon);
        assertEquals(1, pos3.getY(), kTestEpsilon);
        assertEquals(1, pos3.length(), kTestEpsilon);

        // A translation times its inverse should be the identity
/*        Vector2d identity = new Vector2d();
        Pose T1 = new Pose(2.16612, -23.55);
        Pose T2 = T1.changeCoordinateSystem(T1.inverse());
        assertEquals(identity.getX(), T2.getX(), kTestEpsilon);
        assertEquals(identity.getY(), T2.getY(), kTestEpsilon);
        assertEquals(identity.length(), T2.getTranslation().length(), kTestEpsilon);
*/
        // Test interpolation
        pos1 = new Vector2d(0, 1);
        pos2 = new Vector2d(10, -1);
        pos3 = pos1.interpolate(pos2, .5);
        assertEquals(5, pos3.getX(), kTestEpsilon);
        assertEquals(0, pos3.getY(), kTestEpsilon);

        pos1 = new Vector2d(0, 1);
        pos2 = new Vector2d(10, -1);
        pos3 = pos1.interpolate(pos2, .75);
        assertEquals(7.5, pos3.getX(), kTestEpsilon);
        assertEquals(-.5, pos3.getY(), kTestEpsilon);
    }

    @Test
    public void testPose() {
        // Test constructors
        Pose pose1 = new Pose();
        assertEquals(0, pose1.getX(), kTestEpsilon);
        assertEquals(0, pose1.getY(), kTestEpsilon);
        assertEquals(0, pose1.getHeadingDeg(), kTestEpsilon);

        pose1 = new Pose(3, 4, 45*Vector2d.degreesToRadians);
        assertEquals(3, pose1.getX(), kTestEpsilon);
        assertEquals(4, pose1.getY(), kTestEpsilon);
        assertEquals(45, pose1.getHeadingDeg(), kTestEpsilon);

        // Test transformation
        pose1 = new Pose(3, 4, 90*Vector2d.degreesToRadians);
        Pose T = new Pose(1, 0, 0);
        Pose pose3 = pose1.changeCoordinateSystem(T);
        assertEquals(3, pose3.getX(), kTestEpsilon);
        assertEquals(5, pose3.getY(), kTestEpsilon);
        assertEquals(90, pose3.getHeadingDeg(), kTestEpsilon);

        pose1 = new Pose(3, 4,  90*Vector2d.degreesToRadians);
        T = new Pose(1, 0, -90*Vector2d.degreesToRadians);
        pose3 = pose1.changeCoordinateSystem(T);
        assertEquals(3, pose3.getX(), kTestEpsilon);
        assertEquals(5, pose3.getY(), kTestEpsilon);
        assertEquals(0, pose3.getHeadingDeg(), kTestEpsilon);

        // A pose times its inverse should be the identity
/*        Pose identity = new Pose();
        Pose T1 = new Pose(3.51512152, 4.23, 91.6*Vector2d.degreesToRadians);
        Pose T2 = T1.changeCoordinateSystem(T1.inverse());
        assertEquals(identity.getX(), T2.getX(), kTestEpsilon);
        assertEquals(identity.getY(), T2.getY(), kTestEpsilon);
        assertEquals(identity.getRotationDeg(), T2.getRotationDeg(), kTestEpsilon);
*/
        // Test interpolation
        pose1 = new Pose(3, 4, 90*Vector2d.degreesToRadians);
        Pose pose2 = new Pose(13, -6, -90*Vector2d.degreesToRadians);
        pose3 = pose1.interpolate(pose2, .5);
        assertEquals(8, pose3.getX(), kTestEpsilon);
        assertEquals(-1, pose3.getY(), kTestEpsilon);
        assertEquals(0, pose3.getHeadingDeg(), kTestEpsilon);

        pose1 = new Pose(3, 4, 90*Vector2d.degreesToRadians);
        pose2 = new Pose(13, -6, -90*Vector2d.degreesToRadians);
        pose3 = pose1.interpolate(pose2, .75);
        assertEquals(10.5, pose3.getX(), kTestEpsilon);
        assertEquals(-3.5, pose3.getY(), kTestEpsilon);
        assertEquals(-45, pose3.getHeadingDeg(), kTestEpsilon);
    }
}
