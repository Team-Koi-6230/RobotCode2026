package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class MathHelper {
    public static double getClosest(double target, double... options) {
        double closest = options[0];
        double minDistance = Math.abs(target - closest);

        for (double opt : options) {
            double distance = Math.abs(target - opt);
            if (distance < minDistance) {
                minDistance = distance;
                closest = opt;
            }
        }
        return closest;
    }

    public static double getClosestContinuous(double target, double minValue, double maxValue, double... options) {
        if (options == null || options.length == 0) {
            throw new IllegalArgumentException("Options array cannot be empty");
        }

        double range = maxValue - minValue;
        double closest = options[0];
        double minDistance = Double.MAX_VALUE;

        for (double opt : options) {
            double rawDiff = Math.abs(target - opt) % range;

            double distance = Math.min(rawDiff, range - rawDiff);

            if (distance < minDistance) {
                minDistance = distance;
                closest = opt;
            }
        }
        return closest;
    }

    public static double getClosestRadian(double target, double... options) {
        double closest = options[0];
        double minDistance = Double.MAX_VALUE;

        for (double opt : options) {
            double distance = Math.abs(MathUtil.angleModulus(target - opt));

            if (distance < minDistance) {
                minDistance = distance;
                closest = opt;
            }
        }

        return MathUtil.angleModulus(closest);
    }

    public static double getClosestDegree(double target, double... options) {
        double closest = options[0];
        double minDistance = Double.MAX_VALUE;

        for (double opt : options) {
            double distance = Math.abs(MathUtil.inputModulus(target - opt, -180.0, 180.0));

            if (distance < minDistance) {
                minDistance = distance;
                closest = opt;
            }
        }
        return closest;
    }
}
