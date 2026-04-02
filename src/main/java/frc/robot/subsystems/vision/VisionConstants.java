package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;

public class VisionConstants {
    public static final String kLimelightName = "limelight-front";

    public static final double kMaxGyroRateDegPerSec = 720.0;

    public static final double kXYbase = 0.3;
    public static final double kDistExp = 2;
    public static final double kYawHuge = 9_999_999.0;
    public static final double kYawCorrPeriodSec = 0.5;
    public static final double kYawCorrMaxOmegaDeg = 30.0;
    public static final double kYawCorrMaxXYdeltaM = 1.0;
    public static final double kYawCorrMaxYawDeltaDeg = 15.0;
    public static final double kYawCorrSTDDEV = 0.3;

    public static final Transform2d kRobotToLimelightSim = new Transform2d();
}
