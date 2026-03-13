package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesVertical;
import swervelib.SwerveDrive;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {
    private static Vision _instance;

    public static Vision getInstance() {
        if (_instance == null) {
            _instance = new Vision();
        }
        return _instance;
    }

    private final String _limelightName;
    private Pose2d _lastAcceptedPose = new Pose2d();
    private boolean _hasFirstPose = false;
    private double _lastYawCorrTimeSec = -999.0;

    private final MedianFilter velocityFilter = new MedianFilter(5);

    private Vision() {
        _limelightName = VisionConstants.kLimelightName;
    }

    /**
     * Main vision pipeline. Call every loop at the top of
     * SwerveSubsystem.periodic().
     *
     * <p>
     * Internally runs the MT1 yaw correction on its own 0.5 s timer —
     * no separate call is needed.
     *
     * @param swerveDrive YAGSL SwerveDrive instance.
     * @return true if an MT2 measurement was accepted.
     */
    public boolean updatePoseEstimation(SwerveDrive swerveDrive) {
        double headingDeg = swerveDrive.getPose().getRotation().getDegrees();
        double omegaDegPerSec = Math.toDegrees(
                swerveDrive.getRobotVelocity().omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(
                _limelightName,
                headingDeg,
                omegaDegPerSec,
                0.0, 0.0,
                0.0, 0.0);

        // --- Always attempt MT1 yaw correction on its own timer ---
        applyMegaTag1YawCorrection(swerveDrive, omegaDegPerSec);

        // --- Reject if spinning too fast for MT2 ---
        if (Math.abs(omegaDegPerSec) > VisionConstants.kMaxGyroRateDegPerSec) {
            SmartDashboard.putBoolean("Vision/MT2_Rejected_Spin", true);
            return false;
        }
        SmartDashboard.putBoolean("Vision/MT2_Rejected_Spin", false);

        PoseEstimate mt2 = getBotPoseEstimateMegaTag2();

        if (mt2 == null || mt2.tagCount == 0) {
            SmartDashboard.putBoolean("Vision/HasTarget", false);
            return false;
        }
        SmartDashboard.putBoolean("Vision/HasTarget", true);

        // --- Fuse with dynamic stddevs ---
        Matrix<N3, N1> stdDevs = computeDynamicStdDevs(mt2);
        swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, stdDevs);

        // --- Telemetry ---
        SmartDashboard.putNumber("Vision/TagCount", mt2.tagCount);
        SmartDashboard.putNumber("Vision/AvgTagDist_m", mt2.avgTagDist);
        SmartDashboard.putNumber("Vision/XYStdDev", stdDevs.get(0, 0));
        SmartDashboard.putNumber("Vision/PoseX", _lastAcceptedPose.getX());
        SmartDashboard.putNumber("Vision/PoseY", _lastAcceptedPose.getY());
        SmartDashboard.putNumber("Vision/HeadingDeg",
                _lastAcceptedPose.getRotation().getDegrees());

        return true;
    }

    /**
     * Returns the best aim angle for the current state.
     *
     * <p>
     * Uses lead compensation when the robot is moving faster than
     * {@code kMeaningfullVelocity} (your existing constant). Falls back to
     * static aim when nearly stationary.
     *
     * <p>
     * This is the method to pass to {@code driveWhileAiming()} in
     * SwerveSubsystem.
     *
     * @param swerveDrive YAGSL drive for pose + velocity.
     * @return Best-estimate aim {@link Rotation2d} in field-relative frame.
     */
    public Rotation2d getBestAimAngle() {
        ChassisSpeeds fieldVel = Superstructure.getInstance().getDrivebase().getFieldVelocity();
        double speed = Math.hypot(
                fieldVel.vxMetersPerSecond,
                fieldVel.vyMetersPerSecond);

        if (speed > ShooterConstants.kMeaningfullVelocity) {
            return getMovingAimAngle();
        } else {
            return getStaticAimAngle();
        }
    }

    /**
     * Static aim angle — robot-to-hub heading from the fused pose.
     *
     * Works even when no tags are visible (pure odometry). Use this to
     * pre-rotate before the hub enters the camera frame.
     *
     * @param swerveDrive YAGSL drive for pose.
     * @return Rotation2d pointing from robot toward hub center.
     */
    public Rotation2d getStaticAimAngle() {
        Translation2d toTarget = getAllianceHubCenter()
                .minus(getShooterPose().getTranslation());
        return toTarget.getAngle();
    }

    /**
     * Lead-compensated aim angle for shooting while moving.
     *
     * Predicts where the robot will be when the fuel arrives, then aims from
     * that predicted position. Uses your existing {@code kTimeOfFlightMap} —
     * no new constants needed.
     *
     * @param swerveDrive YAGSL drive for pose + field velocity.
     * @return Lead-corrected Rotation2d in field-relative frame.
     */
    public Rotation2d getMovingAimAngle() {
        var swerveDrive = Superstructure.getInstance().getDrivebase();
        Translation2d robotPos = getShooterPose().getTranslation();
        Translation2d target = getAllianceHubCenter();

        ChassisSpeeds fv = swerveDrive.getFieldVelocity();
        double smoothedVx = velocityFilter.calculate(fv.vxMetersPerSecond);
        double smoothedVy = velocityFilter.calculate(fv.vyMetersPerSecond);
        Translation2d velocity = new Translation2d(
                smoothedVx, smoothedVy);

        double dist = robotPos.getDistance(target);
        double flightTime = ShooterConstants.kTimeOfFlightMap.get(dist);

        Translation2d predicted = robotPos.plus(velocity.times(flightTime));
        Translation2d toTarget = target.minus(predicted);

        SmartDashboard.putNumber("Vision/Aim_DistM", dist);
        SmartDashboard.putNumber("Vision/Aim_FlightTimeSec", flightTime);
        SmartDashboard.putNumber("Vision/Aim_LeadX", predicted.getX());
        SmartDashboard.putNumber("Vision/Aim_LeadY", predicted.getY());

        return toTarget.getAngle();
    }

    /**
     * Distance-based flywheel RPM with radial velocity compensation.
     *
     * Combines:
     * 1. {@code kShotFlywheelSpeedMap} (distance → RPM)
     * 2. {@code kRadialRPMComp} — the constant that existed in your code
     * but was never wired in. Now it is.
     *
     * Moving toward hub → less RPM needed (ball already has robot's momentum).
     * Moving away from hub → more RPM needed to compensate velocity loss.
     *
     * @param swerveDrive YAGSL drive for pose + velocity.
     * @return Compensated RPM setpoint.
     */
    public double getCompensatedFlywheelRPM() {
        Translation2d robotPos = getShooterPose().getTranslation();
        Translation2d target = getAllianceHubCenter();
        double dist = robotPos.getDistance(target);

        double baseRPM = ShooterConstants.kShotFlywheelSpeedMap.get(dist);

        // Unit vector from robot → hub
        double hubDist = Math.max(0.01, dist);
        Translation2d toHub = target.minus(robotPos);
        double ux = toHub.getX() / hubDist;
        double uy = toHub.getY() / hubDist;

        // Dot robot field-velocity onto the robot→hub axis.
        // Positive = moving toward hub, negative = moving away.
        ChassisSpeeds fv = Superstructure.getInstance().getDrivebase().getFieldVelocity();
        double radialTowardHub = fv.vxMetersPerSecond * ux + fv.vyMetersPerSecond * uy;

        // Subtract when moving toward (less power needed), add when moving away.
        double radialComp = -(radialTowardHub / SwerveDriveConstants.kMaxSpeed)
                * ShooterConstants.kRadialRPMComp;

        SmartDashboard.putNumber("Vision/RPM_Base", baseRPM);
        SmartDashboard.putNumber("Vision/RPM_RadialComp", radialComp);
        SmartDashboard.putNumber("Vision/RPM_Compensated", baseRPM + radialComp);
        SmartDashboard.putNumber("Vision/RPM_RadialSpeed", radialTowardHub);

        return baseRPM + radialComp;
    }

    /**
     * Hood angle (degrees) from {@code kShotHoodAngleMap} for current distance.
     *
     * @param swerveDrive YAGSL drive for pose.
     * @return Hood servo angle in degrees.
     */
    public double getHoodAngle() {
        double dist = Superstructure.getInstance().getDrivebase().getPose().getTranslation()
                .getDistance(getAllianceHubCenter());
        return ShooterConstants.kShotHoodAngleMap.get(dist);
    }

    /**
     * Straight-line distance (meters) to the alliance hub center.
     *
     * <p>
     * NOTE: This replaces the old {@code getDistanceToHub()} which returned
     * a Translation2d. SwerveSubsystem's {@code getHubRelativeVelocity()} and
     * the SmartDashboard call in periodic() must be updated to use this.
     *
     * @param swerveDrive YAGSL drive for pose.
     * @return Distance in meters.
     */
    public double getDistanceToHub() {
        return getShooterPose().getTranslation()
                .getDistance(getAllianceHubCenter());
    }

    /**
     * Whether the robot is currently in its own alliance zone.
     *
     * REBUILT is rotationally symmetric. AllianceFlipUtil.applyX() correctly
     * mirrors the zone-line x-coordinate for red alliance.
     */
    public boolean isInAllianceZone() {
        double x = AllianceFlipUtil.applyX(_lastAcceptedPose.getX());
        return x < LinesVertical.allianceZone;
    }

    /**
     * Alliance-aware hub center using AllianceFlipUtil.
     *
     * REBUILT is rotationally symmetric, so apply() (which flips both X and Y)
     * correctly maps the blue hub center to the red hub center.
     */
    public Translation2d getAllianceHubCenter() {
        return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint)
                .toTranslation2d();
    }

    /** Last pose accepted by the vision estimator (for logging). */
    public Pose2d getLastAcceptedPose() {
        return _lastAcceptedPose;
    }

    /**
     * Periodic MegaTag1 yaw correction for NavX drift compensation.
     *
     * <p>
     * MT1 solves full 6-DOF independently from the gyro. With 2+ tags its
     * yaw estimate is reliable enough to use as a drift check. We inject it
     * as a vision measurement with huge XY stddev (Kalman ignores XY position)
     * and small yaw stddev (Kalman gently pulls heading toward MT1).
     *
     * <p>
     * This fires at most once per {@code YAW_CORR_PERIOD_SEC} and only when
     * all gate conditions pass. Called internally from updatePoseEstimation().
     *
     * @param swerveDrive    YAGSL drive instance.
     * @param omegaDegPerSec Current angular velocity (from wheel kinematics).
     */
    private void applyMegaTag1YawCorrection(SwerveDrive swerveDrive,
            double omegaDegPerSec) {
        double now = Timer.getFPGATimestamp();
        if (now - _lastYawCorrTimeSec < VisionConstants.kYawCorrPeriodSec)
            return;

        // robot must not be spinning (MT1 needs a stable image)
        if (Math.abs(omegaDegPerSec) > VisionConstants.kYawCorrMaxOmegaDeg) {
            SmartDashboard.putBoolean("Vision/YawCorr_Applied", false);
            return;
        }

        PoseEstimate mt1 = getBotPoseEstimateMegaTag1();

        // need at least 2 tags to eliminate MT1 pose ambiguity
        if (mt1 == null || mt1.tagCount < 2) {
            SmartDashboard.putBoolean("Vision/YawCorr_Applied", false);
            return;
        }

        Pose2d currentPose = swerveDrive.getPose();

        // XY positions must broadly agree (not a false-positive tag)
        double xyDelta = mt1.pose.getTranslation()
                .getDistance(currentPose.getTranslation());
        if (xyDelta > VisionConstants.kYawCorrMaxXYdeltaM) {
            SmartDashboard.putBoolean("Vision/YawCorr_Applied", false);
            SmartDashboard.putNumber("Vision/YawCorr_XYDelta_m", xyDelta);
            return;
        }

        // yaw delta must be small enough to be drift, not a bad reading
        double mt1Yaw = mt1.pose.getRotation().getDegrees();
        double odoYaw = currentPose.getRotation().getDegrees();
        double yawDelta = Math.abs(mt1Yaw - odoYaw);
        if (yawDelta > 180.0)
            yawDelta = 360.0 - yawDelta; // wrap to [0, 180]

        if (yawDelta > VisionConstants.kYawCorrMaxYawDeltaDeg) {
            SmartDashboard.putBoolean("Vision/YawCorr_Applied", false);
            SmartDashboard.putNumber("Vision/YawCorr_YawDelta_deg", yawDelta);
            return;
        }

        swerveDrive.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds,
                VecBuilder.fill(VisionConstants.kYawHuge, VisionConstants.kYawHuge, VisionConstants.kYawCorrSTDDEV));

        _lastYawCorrTimeSec = now;

        SmartDashboard.putBoolean("Vision/YawCorr_Applied", true);
        SmartDashboard.putNumber("Vision/YawCorr_YawDelta_deg", yawDelta);
        SmartDashboard.putNumber("Vision/YawCorr_XYDelta_m", xyDelta);
        SmartDashboard.putNumber("Vision/YawCorr_MT1Yaw_deg", mt1Yaw);
    }

    private Matrix<N3, N1> computeDynamicStdDevs(PoseEstimate mt2) {
        double tagCount = Math.max(1.0, mt2.tagCount);
        double dist = Math.max(0.5, mt2.avgTagDist);

        double xyStdDev = VisionConstants.kXYbase
                * Math.pow(dist, VisionConstants.kDistExp)
                * (1.0 / Math.sqrt(tagCount));

        if (tagCount >= 2)
            xyStdDev *= 0.5;

        return VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.kYawHuge);
    }

    private PoseEstimate getBotPoseEstimateMegaTag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);
    }

    private PoseEstimate getBotPoseEstimateMegaTag1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(_limelightName);
    }

public Pose2d getShooterPose() {
    var drivebase = Superstructure.getInstance().getDrivebase();
    Pose2d robotPose = drivebase.getPose();

    Transform2d shooterTransform = new Transform2d(
            ShooterConstants.kShooterOffset.getX(),
            ShooterConstants.kShooterOffset.getY(),
            new Rotation2d());

    return robotPose.transformBy(shooterTransform);
}
}
