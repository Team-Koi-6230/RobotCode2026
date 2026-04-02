package frc.robot.subsystems.vision.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean mt1HasTarget = false;
        public Pose2d mt1Pose = new Pose2d();
        public double mt1Timestamp = 0.0;
        public int mt1TagCount = 0;

        public boolean mt2HasTarget = false;
        public Pose2d mt2Pose = new Pose2d();
        public double mt2Timestamp = 0.0;
        public int mt2TagCount = 0;
        public double mt2AvgTagDist = 0.0;
    }

    public default void updateInputs(VisionIOInputs inputs) {
    }

    public default void setRobotOrientation(double headingDeg, double omegaDegPerSec) {
    }
}