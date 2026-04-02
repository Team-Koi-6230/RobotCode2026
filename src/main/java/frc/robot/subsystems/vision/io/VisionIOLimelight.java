package frc.robot.subsystems.vision.io;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

public class VisionIOLimelight implements VisionIO {
    private final String limelightName;

    public VisionIOLimelight() {
        this.limelightName = VisionConstants.kLimelightName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Read MegaTag 1
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 != null && mt1.tagCount > 0) {
            inputs.mt1HasTarget = true;
            inputs.mt1Pose = mt1.pose;
            inputs.mt1Timestamp = mt1.timestampSeconds;
            inputs.mt1TagCount = mt1.tagCount;
        } else {
            inputs.mt1HasTarget = false;
        }

        // Read MegaTag 2
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 != null && mt2.tagCount > 0) {
            inputs.mt2HasTarget = true;
            inputs.mt2Pose = mt2.pose;
            inputs.mt2Timestamp = mt2.timestampSeconds;
            inputs.mt2TagCount = mt2.tagCount;
            inputs.mt2AvgTagDist = mt2.avgTagDist;
        } else {
            inputs.mt2HasTarget = false;
        }
    }

    @Override
    public void setRobotOrientation(double headingDeg, double omegaDegPerSec) {
        LimelightHelpers.SetRobotOrientation(
                limelightName,
                headingDeg,
                omegaDegPerSec,
                0.0, 0.0,
                0.0, 0.0);
    }
}