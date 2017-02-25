package org.team686.simsbot;

/**
 * Attribution: adapted from FRC Team 254
 */

import org.team686.lib.util.ConstantsBase;

import edu.wpi.first.wpilibj.I2C;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase
{
    public static double kLoopDt = 0.01;

	// Wheels
    public static double kDriveWheelCircumInches = 13.00;
    public static double kDriveWheelDiameterInches = kDriveWheelCircumInches / Math.PI;
    public static double kTrackLengthInches = 9.625;
    public static double kTrackWidthInches = 25.125;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = 0.5;

    // Wheel Encoder
    public static int kQuadEncoderCodesPerRev = 256;
    public static int kQuadEncoderPulsesPerRev = 4*kQuadEncoderCodesPerRev;
    public static double kQuadEncoderStatusFramePeriod = 0.100;	// 100ms
    
    // CONTROL LOOP GAINS
    public static double kFullThrottleRPM = 520;	// measured max RPM using NI web interface
    public static double kFullThrottleEncoderPulsePer100ms = kFullThrottleRPM / 60.0 * kQuadEncoderStatusFramePeriod * kQuadEncoderPulsesPerRev; 
    
    // PID gains for drive velocity loop (sent to Talon)
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 1.0;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 6.0;
    public static double kDriveVelocityKf = 1023.0 / kFullThrottleEncoderPulsePer100ms;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4*256 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 0.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 0.0;

    // Path following constants
    public static double kPathFollowingMaxVel    = 60.0; // inches/sec  		
    public static double kPathFollowingMaxAccel  = 48.0; // inches/sec^2	
    public static double kPathFollowingLookahead = 24.0; // inches
    public static double kPathFollowingCompletionTolerance = 1.0; 
    
    // Vision constants
    public static double kVisionMaxVel    = 60.0; // inches/sec  		// RS measured ~100 inches/sec on carpet
    public static double kVisionMaxAccel  = 48.0; // inches/sec^2		// RS measured 800-1000 inches/sec^2 on carpet
    public static double kTargetWidthInches = 10.25;
    public static double kTargetLocationFilterConstant = (30.0 * kLoopDt);		// 30 time constants in 1 second
    public static double kCameraFOVDegrees = 42.5;			// Camera Field of View (degrees)
    public static double kCameraHalfFOVRadians = kCameraFOVDegrees/2.0 * Math.PI/180.0;			// Half of Camera Field of View (radians)
    public static double kTangentCameraHalfFOV = Math.tan(kCameraHalfFOVRadians);
    public static double kCameraLatencySeconds = 0.200;			// Camera image capturing latency
    public static double kPegTargetDistanceThresholdInches = 24;		// inches to stop from target (15" from camera = 4.5" from bumpers
    public static double kVisionMaxDistanceInches = 240;		// ignore targets greater than this distance
    public static double kVisionLookaheadDist = 24.0;	// inches
    
    // Do not change anything after this line!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1O2Szvp3cC3gO2euKjqhdmpsyd54t6eB2t9jzo41G2H4
    // Talons
    // (Note that if multiple Talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    // TALONS
    public static final int kLeftMotorTalonId  = 1;
    public static final int kRightMotorTalonId = 2;
    
    public static int kXboxButtonA  = 1;
    public static int kXboxButtonB  = 2;
    public static int kXboxButtonX  = 3;
    public static int kXboxButtonY  = 4;
    public static int kXboxButtonLB = 5;
    public static int kXboxButtonRB = 6;
    
    public static int kXboxLStickXAxis  = 0;
    public static int kXboxLStickYAxis  = 1;
    public static int kXboxLTriggerAxis = 2;
    public static int kXboxRTriggerAxis = 3;
    public static int kXboxRStickXAxis  = 4;
    public static int kXboxRStickYAxis  = 5;

	// The I2C port the BNO055 is connected to
    public static final I2C.Port BNO055_PORT = I2C.Port.kOnboard;
    
    // BNO055 accelerometer calibration constants
    // ( -7, -34,  33, -24) - taken 10/14/2016
    // (-13, -53,  18, -24) - taken 10/14/2016
    // (  0, -59,  25, -24) - taken 10/14/2016
    // using average of the above
    public static short kAccelOffsetX =  -7;
    public static short kAccelOffsetY = -53;
    public static short kAccelOffsetZ =  25;
    public static short kAccelRadius  = -24;
    
    
}
