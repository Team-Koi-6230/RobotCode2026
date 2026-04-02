package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.io.VisionIO;
import frc.robot.subsystems.vision.io.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.io.VisionIOLimelight;
import frc.robot.subsystems.vision.io.VisionIOSim;

// no need for Upstream here, no superstate relation.
public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private Pose2d _lastAcceptedPose = new Pose2d();
    private double _lastYawCorrTimeSec = -999.0;

    public Vision() {
        this.io = getIO();
    }

    @Override
    public void periodic() {
        // Automatically syncs the hardware data to the inputs object and logs it
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
    }

    /**
     * Main vision pipeline. Call every loop AFTER the drivetrain has updated its
     * odometry.
     *
     * @return true if an MT2 measurement was accepted.
     */
    public boolean updatePoseEstimation() {
        double headingDeg = RobotContainer.getRobotPose().getRotation().getDegrees();
        double omegaDegPerSec = Math.toDegrees(RobotContainer.getRobotVelocity().omegaRadiansPerSecond);

        // Send data TO the hardware layer for the Limelight to use on its next frame
        io.setRobotOrientation(headingDeg, omegaDegPerSec);

        // --- Always attempt MT1 yaw correction on its own timer ---
        applyMegaTag1YawCorrection(omegaDegPerSec);

        Logger.recordOutput("Vision/Omega", omegaDegPerSec);

        // --- Reject if spinning too fast for MT2 ---
        if (Math.abs(omegaDegPerSec) > VisionConstants.kMaxGyroRateDegPerSec) {
            Logger.recordOutput("Vision/MT2_Rejected_Spin", true);
            return false;
        }
        Logger.recordOutput("Vision/MT2_Rejected_Spin", false);

        // --- Check logged inputs for MT2 data ---
        if (!inputs.mt2HasTarget) {
            Logger.recordOutput("Vision/HasTarget", false);
            return false;
        }
        Logger.recordOutput("Vision/HasTarget", true);

        // --- Fuse with dynamic stddevs ---
        Matrix<N3, N1> stdDevs = computeDynamicStdDevs();
        RobotContainer.addVisionMeasurement(inputs.mt2Pose, inputs.mt2Timestamp, stdDevs);

        _lastAcceptedPose = inputs.mt2Pose;

        // --- Telemetry ---
        Logger.recordOutput("Vision/TagCount", inputs.mt2TagCount);
        Logger.recordOutput("Vision/AvgTagDist_m", inputs.mt2AvgTagDist);
        Logger.recordOutput("Vision/XYStdDev", stdDevs.get(0, 0));
        Logger.recordOutput("Vision/LastAcceptedPose", _lastAcceptedPose);

        return true;
    }

    private void applyMegaTag1YawCorrection(double omegaDegPerSec) {
        double now = Timer.getFPGATimestamp();
        if (now - _lastYawCorrTimeSec < VisionConstants.kYawCorrPeriodSec)
            return;

        // Robot must not be spinning
        if (Math.abs(omegaDegPerSec) > VisionConstants.kYawCorrMaxOmegaDeg) {
            Logger.recordOutput("Vision/YawCorr_Applied", false);
            return;
        }

        // Need at least 2 tags to eliminate MT1 pose ambiguity
        if (!inputs.mt1HasTarget || inputs.mt1TagCount < 2) {
            Logger.recordOutput("Vision/YawCorr_Applied", false);
            return;
        }

        Pose2d currentPose = RobotContainer.getRobotPose();

        // XY positions must broadly agree
        double xyDelta = inputs.mt1Pose.getTranslation().getDistance(currentPose.getTranslation());
        if (xyDelta > VisionConstants.kYawCorrMaxXYdeltaM) {
            Logger.recordOutput("Vision/YawCorr_Applied", false);
            Logger.recordOutput("Vision/YawCorr_XYDelta_m", xyDelta);
            return;
        }

        // Yaw delta must be small enough to be drift
        double mt1Yaw = inputs.mt1Pose.getRotation().getDegrees();
        double odoYaw = currentPose.getRotation().getDegrees();
        double yawDelta = Math.abs(mt1Yaw - odoYaw);
        if (yawDelta > 180.0)
            yawDelta = 360.0 - yawDelta;

        if (yawDelta > VisionConstants.kYawCorrMaxYawDeltaDeg) {
            Logger.recordOutput("Vision/YawCorr_Applied", false);
            Logger.recordOutput("Vision/YawCorr_YawDelta_deg", yawDelta);
            return;
        }

        // Apply correction
        RobotContainer.addVisionMeasurement(
                inputs.mt1Pose,
                inputs.mt1Timestamp,
                VecBuilder.fill(VisionConstants.kYawHuge, VisionConstants.kYawHuge, VisionConstants.kYawCorrSTDDEV));

        _lastYawCorrTimeSec = now;

        Logger.recordOutput("Vision/YawCorr_Applied", true);
        Logger.recordOutput("Vision/YawCorr_YawDelta_deg", yawDelta);
        Logger.recordOutput("Vision/YawCorr_XYDelta_m", xyDelta);
        Logger.recordOutput("Vision/YawCorr_MT1Yaw_deg", mt1Yaw);
    }

    private Matrix<N3, N1> computeDynamicStdDevs() {
        double tagCount = Math.max(1.0, inputs.mt2TagCount);
        double dist = Math.max(0.5, inputs.mt2AvgTagDist);

        double xyStdDev = VisionConstants.kXYbase
                * Math.pow(dist, VisionConstants.kDistExp)
                * (1.0 / Math.sqrt(tagCount));

        if (tagCount >= 2)
            xyStdDev *= 0.5;

        return VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.kYawHuge);
    }

    private VisionIO getIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new VisionIOLimelight();

            case SIM:
                return new VisionIOSim(RobotContainer::getRobotPose, VisionConstants.kRobotToLimelightSim);

            case REPLAY:
                return new VisionIO() {
                };
        }
        return new VisionIO() {
        };
    }
}