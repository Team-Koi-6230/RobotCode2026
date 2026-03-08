package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.RumbleSubsystem.Priority;
import frc.robot.utils.RumblePack;

public final class Constants {
  public static boolean disableHAL = false;
  public static double loopPeriodSecs = 0.02;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDeadband = 0.3;
    public static final RumblePack kGameShiftRumble = new RumblePack(0.7, 0.5, Priority.HIGH);
    public static final int kTeleopInterval = 25;
    public static final int kEndGameTime = 30;
  }

  public static class VisionConstants {
    public static final String kLimelightName = "limelight-front";

    public static final double kAimTolerance = Math.toRadians(1.0);
    public static final double kMaxGyroRateDegPerSec = 720.0;
    public static final double kMaxPoseJumpMeters = 1.0;

    public static final double kXYbase = 0.5;
    public static final double kDistExp = 0.8;
    public static final double kYawHuge = 9_999_999.0;
    public static final double kYawCorrPeriodSec = 0.5;
    public static final double kYawCorrMaxOmegaDeg = 30.0;
    public static final double kYawCorrMaxXYdeltaM = 0.30;
    public static final double kYawCorrMaxYawDeltaDeg = 8.0;
    public static final double kYawCorrSTDDEV = 0.3;
  }

  public static class SwerveDriveConstants {
    public static final double kMaxSpeed = 5.36448;
    public static final double kMaxAngularVelocity = 4 * Math.PI;
    public static final double kAimingSpeedModifier = 2.5;
    public static final double kMaxStrafe = 2;

    public static final double kTargetErrorToleranceStatic = Math.toRadians(1.5);
    public static final double kTargetErrorToleranceMoving = Math.toRadians(3.0);

    // PID Gains
    public static final double kPr = 10.0, kIr = 0, kDr = 0.75;
    public static final double kPrMoving = 36.0, kIrMoving = 0, kDrMoving = 1;

    public static final double kSr = 0.1;
    public static final double kVr = 0.40;
    public static final double kAr = 0.01;

    public static final double kMaxRotVelocity = 8 * Math.PI; 
    public static final double kMaxRotAcceleration = 16 * Math.PI;
  }

  public static class PathPlanner {
    public static final double kTranslationP = 5, kTranslationI = 0, kTranslationD = 0;
    public static final double kRotationP = 5, kRotationI = 0, kRotationD = 0;
    public static final double kClimbTimer = 1.5;
  }

  public static class ShooterConstants {
    public static final int kMainMotorID = 21;
    public static final int kSecondaryMotorID = 20;

    public static final double kGearRatio = 1.0;
    public static final double kTolerance = 200;
    public static final int kStallLimit = 80;
    public static final boolean kInverted = true;

    public static final double kP = 0.00008,
        kI = 0,
        kD = 0,
        kS = 0.01,
        kV = 0.0018,
        kA = 0;

    public static final double kNeutralZoneShootingRPM = 6000;

    public static final InterpolatingDoubleTreeMap kShotHoodAngleMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kShotFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap kTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static void fillMap(InterpolatingDoubleTreeMap map,
        double[] distances, double[] values) {
      if (distances.length != values.length)
        throw new IllegalArgumentException("Distances and values must be same length");
      for (int i = 0; i < distances.length; i++)
        map.put(distances[i], values[i]);
    }

    static {
      fillMap(kShotHoodAngleMap,
          new double[] { 1.3, 2.0, 3.0, 4.0, 5.5 },
          new double[] { 18.0, 22.0, 27.0, 31.0, 35.0 });

      fillMap(kShotFlywheelSpeedMap,
          new double[] { 1.3, 2.0, 3.0, 4.0, 5.5 },
          new double[] { 3000, 4500, 5000, 5700, 6250 });

      fillMap(kTimeOfFlightMap,
          new double[] { 1.3, 2.0, 3.0, 4.0, 5.5 },
          new double[] { 0.3, 0.57, 0.78, 0.9, 1 });
    }

    public static final double kMaxShootingDist = 4.0;

    public static final double kRadialRPMComp = 150;

    public static final double kSidewaysVelocityDeadband = 0.15;
    public static final double kMeaningfullVelocity = 0.5;

    public static final RumblePack kRumbleScoreReady = new RumblePack(0.3, 0.2, Priority.MEDIUM);
    public static final double kUnjamPower = -2000;
    public static final double kMaxRPM = 6250;

    public static final Transform3d kShooterOffset = new Transform3d(0.05, 0, 0.45, new Rotation3d(0, 0, 0));
  }

  public static class HoodConstants {
    public static final int kServoRightID = 8;
    public static final int kServoLeftID = 9;
    public static final double kMinDeg = 0;
    public static final double kMaxDeg = 180;
    public static final int kServoMin = 0;
    public static final int kServoMax = 180;
    public static final int kStartingPos = 0;
    public static final double kServoDelay = 0.04;
    public static final double kAllianceAngle = 0.0;
  }

  public static class IntakeArmConstants {
    public static final int kAbsoluteEncoderID = 0;
    public static final int kMotorID = 15;
    public static final double kP = 0.0001, kI = 0, kD = 0;
    public static final double kS = 0.144, kV = 0.0025, kA = 0, kG = 0.185;
    public static final double kCosRatio = 1;
    public static final double kMaxAcceleration = 46000;
    public static final double kCruiseVelocity = 400000;
    public static final double kGearRatio = 25;
    public static final int kAbsoluteEncoderRange = 360;
    public static final int kAbsoluteEncoderOffset = 160;
    public static final double kTolerance = 0.5;
    public static final int kOpenAngle = 323;
    public static final int kClosedAngle = 7;
    public static final double KStepDistance = 100;
    public static final double kShakeMax = kOpenAngle - 3;
    public static final double kShakeMin = kClosedAngle + 3;
    public static final double kForwardSoftLimit = 324;
    public static final double kReverseSoftLimit = 6;
  }

  public static class IntakeRollerConstants {
    public static final int kMotorID = 18;
    public static final double kIntakePower = 1400;
    public static final double kShootingPower = 460;
    public static final RumblePack kIntakeReadyRumble = new RumblePack(0.3, 0.2, Priority.LOW);
    public static final int kStallLimit = 40;
    public static final double kP = 0, kI = 0, kD = 0, kS = 0.18, kV = 0.00835, kA = 0;
    public static final double kGearRatio = 4;
  }

  public static class FeederConstants {
    public static final int kMotorID = 16;
    public static final double kGrabRpm = 1500;
    public static final int kStallLimit = 60;
    public static final double kGearRatio = 4;
    public static final double kP = 0.01, kI = 0, kD = 0, kS = 0.150, kV = 0.5, kA = 0;
  }

  public final class ClimberConstants {
    public static final double kS_ground = 0.0, kG_ground = 0.0;
    public static final double kV_ground = 0.0, kA_ground = 0.0;
    public static final double kP_ground = 0.0, kI_ground = 0.0, kD_ground = 0.0;
    public static final double kS_hang = 0.0, kG_hang = 0.0;
    public static final double kV_hang = 0.0, kA_hang = 0.0;
    public static final double kP_hang = 0.0, kI_hang = 0.0, kD_hang = 0.0;
    public static final double kTolerance = 0.003;
    public static final double kMaxAcceleration = 0, kMaxVelocity = 0;
    public static final int kMainMotorID = 19, kSecondaryMotorID = 20;
    public static final int kDutyCycleChannel = 1, kDutyCycleOffset = 0;
    public static final int kStallLimit = 40;
    public static final double kMetersPerRotation = 0.0;
    public static final double kClosedElevator = 0.0;
    public static final double kL1ExtendTeleop = 0,
                               kL1ExtendAuton = 0,
                               kL1CloseTeleop = 0,
                               kL1CloseAuton = 0;
  }

  public static class LEDconstants {
    public static final int kLedPort = 2;
    public static final int kLedCount = 120;
    public static final double kBlinkTime = 0.5;
    public static final LEDPattern kIdleLED = LEDPattern.solid(new Color("#800080"));
    public static final LEDPattern kPrepLED = LEDPattern.solid(new Color("#FFEE8C"));
    public static final LEDPattern kShootLED = LEDPattern.solid(new Color("rgba(13, 165, 165, 1)"));
    public static final LEDPattern kIntakeLED = LEDPattern.solid(new Color("rgba(255, 65, 138, 1)"));
    public static final LEDPattern kClimbsLED = LEDPattern.solid(new Color("rgba(45, 68, 243, 1)"));
  }
} 