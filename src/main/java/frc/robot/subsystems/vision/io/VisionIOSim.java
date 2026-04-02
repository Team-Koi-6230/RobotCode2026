package frc.robot.subsystems.vision.io;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOSim implements VisionIO {
    private final Supplier<Pose2d> truePoseSupplier;
    private final Transform2d robotToCamera;
    private AprilTagFieldLayout fieldLayout;

    // Simulated Camera Properties (Adjust to match your physical Limelight)
    private static final double MAX_VISIBLE_DISTANCE_M = 6.0;
    private static final double FOV_HORIZONTAL_DEGREES = 63.3; // Limelight 3/4 rough FOV
    private static final double SIM_BASE_NOISE_M = 0.05; // Base positional noise

    private final Random random = new Random();

    /**
     * @param truePoseSupplier A supplier that returns the EXACT simulated pose of
     *                         the robot.
     * @param robotToCamera    The 2D transform from the center of the robot to the
     *                         camera lens.
     */
    public VisionIOSim(Supplier<Pose2d> truePoseSupplier, Transform2d robotToCamera) {
        this.truePoseSupplier = truePoseSupplier;
        this.robotToCamera = robotToCamera;

        try {
            // Load the current year's game map
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (Exception e) {
            e.printStackTrace();
            fieldLayout = null; // Fallback if file isn't found
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (fieldLayout == null)
            return;

        Pose2d trueRobotPose = truePoseSupplier.get();
        Pose2d trueCameraPose = trueRobotPose.transformBy(robotToCamera);

        List<AprilTag> visibleTags = new ArrayList<>();
        double totalDistance = 0.0;

        // 1. Calculate Visibility
        for (AprilTag tag : fieldLayout.getTags()) {
            Pose2d tagPose = tag.pose.toPose2d();

            // Calculate distance to tag
            double distance = trueCameraPose.getTranslation().getDistance(tagPose.getTranslation());
            if (distance > MAX_VISIBLE_DISTANCE_M)
                continue;

            // Calculate angle to tag relative to the camera's lens
            Translation2d toTag = tagPose.getTranslation().minus(trueCameraPose.getTranslation());
            Rotation2d angleToTagFieldRelative = toTag.getAngle();
            Rotation2d angleToTagCameraRelative = angleToTagFieldRelative.minus(trueCameraPose.getRotation());

            // Check if tag is within the horizontal Field of View
            if (Math.abs(angleToTagCameraRelative.getDegrees()) < (FOV_HORIZONTAL_DEGREES / 2.0)) {
                visibleTags.add(tag);
                totalDistance += distance;
            }
        }

        // 2. Populate Inputs
        int tagCount = visibleTags.size();
        if (tagCount > 0) {
            double avgDist = totalDistance / tagCount;

            // Calculate noise scale: further away and fewer tags = more noise
            double noiseScale = SIM_BASE_NOISE_M * (avgDist / 2.0) * (1.0 / Math.sqrt(tagCount));

            // Generate a noisy pose based on the true pose
            Pose2d noisyPose = new Pose2d(
                    trueRobotPose.getX() + (random.nextGaussian() * noiseScale),
                    trueRobotPose.getY() + (random.nextGaussian() * noiseScale),
                    trueRobotPose.getRotation()
                            .plus(Rotation2d.fromDegrees(random.nextGaussian() * (noiseScale * 10))));

            double timestamp = Timer.getFPGATimestamp() - 0.02; // Simulate ~20ms pipeline latency

            // Populate MegaTag 1 (In sim, we just give it the noisy pose)
            inputs.mt1HasTarget = true;
            inputs.mt1TagCount = tagCount;
            inputs.mt1Timestamp = timestamp;
            inputs.mt1Pose = noisyPose;

            // Populate MegaTag 2 (In reality MT2 uses gyro, so in sim we'll make its
            // heading perfectly match the true pose)
            inputs.mt2HasTarget = true;
            inputs.mt2TagCount = tagCount;
            inputs.mt2AvgTagDist = avgDist;
            inputs.mt2Timestamp = timestamp;
            inputs.mt2Pose = new Pose2d(noisyPose.getTranslation(), trueRobotPose.getRotation());

        } else {
            // No tags visible
            inputs.mt1HasTarget = false;
            inputs.mt2HasTarget = false;
            inputs.mt1TagCount = 0;
            inputs.mt2TagCount = 0;
            inputs.mt2AvgTagDist = 0.0;
        }
    }

    @Override
    public void setRobotOrientation(double headingDeg, double omegaDegPerSec) {
        // Do nothing. In simulation, we already have access to the true simulated
        // heading.
    }
}