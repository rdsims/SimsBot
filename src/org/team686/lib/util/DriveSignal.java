package org.team686.lib.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether
 * the brake mode is enabled.
 */
public class DriveSignal {
    public double lMotor;
    public double rMotor;
    public boolean brakeMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        this.lMotor = left;
        this.rMotor = right;
        this.brakeMode = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0, false);
    public static DriveSignal BRAKE   = new DriveSignal(0, 0, true);

    @Override
    public String toString() {
        return "L: " + lMotor + ", R: " + rMotor;
    }
}
