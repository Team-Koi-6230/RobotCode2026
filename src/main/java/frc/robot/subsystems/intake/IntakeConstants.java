package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

    public static final double  kRollerGearRatio                = 4;
    public static final double  kPivotMotorToShaftGearRatio     = 25;
    public static final double  kPivotMotorToPivotGearRatio     = 75;
    public static final double  kPivotShaftToPivotGearRatio     = kPivotMotorToPivotGearRatio / kPivotMotorToShaftGearRatio;

    public static final int     kPivotCurrentLimits             = 20;
    public static final int     kRollerCurrentLimits            = 40;
    public static final int     kIntakingVolts                  = 12;
    public static final int     kShootingVolts                  = 6;
    public static final boolean kMotorInverted                  = true;

    public static final double  kThroughBoreRange               = 360;
    public static final double  kThroughBoreOffset              = 160;
    public static final boolean kEncoderInverted                = true;

    public static final double  kForwardSoftLimit               = 324;
    public static final double  kReverseSoftLimit               = 6;

    public static final Rotation2d kMinAngle                    = Rotation2d.fromDegrees(0);
    public static final Rotation2d kMaxAngle                    = Rotation2d.fromDegrees(100);
    public static final Rotation2d kClosedAngle                 = Rotation2d.fromDegrees(95);
    public static final Rotation2d kOpenAngle                   = Rotation2d.fromDegrees(1);
    public static final Rotation2d kMinOpenAngle                = Rotation2d.fromDegrees(10);
    public static final double  kErrorToleranceDeg              = 2;

    public static final double  kWantedSteps                    = 3;
    public static final double  kStepSizeDegrees                = kClosedAngle.getDegrees() / 3;

    public static final double  kMOIpivot                       = 0.1;
    public static final double  kLengthPivot                    = 0.3;

    public static final double  kP                              = 0.0001;
    public static final double  kI                              = 0;
    public static final double  kD                              = 0;
    public static final double  kS                              = 0.144;
    public static final double  kV                              = 0.0025;
    public static final double  kA                              = 0;
    public static final double  kG                              = 0.185;
    public static final double  kCosRatio                       = 1;
    public static final double  kMaxAcceleration                = 46000;
    public static final double  kCruiseVelocity                 = 400000;
    public static final double  kTolerance                      = 0.5;

    public static final double  kPsim                           = 6;
    public static final double  kIsim                           = 0;
    public static final double  kDsim                           = 0.08;
    public static final double  kSsim                           = 0.09;
    public static final double  kGsim                           = 0.3;
    public static final double  kVsim                           = 0.18;
    public static final double  kAsim                           = 0.001;
    public static final double  kMaxVelocityRadPerSec           = 16 * Math.PI;
    public static final double  kMaxAccelRadPerSecSquared       = 120 * Math.PI;
}