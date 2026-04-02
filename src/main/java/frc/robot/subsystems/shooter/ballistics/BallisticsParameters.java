package frc.robot.subsystems.shooter.ballistics;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class BallisticsParameters {
    public static final Transform3d kShooterOffset = new Transform3d(0.05, 0, 0.45, new Rotation3d(0, 0, 0));

    public static final double kMaxSpeed = 32.725;

    public static final double kPassingSpeed = 18;
    public static final double kPassingAngle = 0;

    public static final double kFlywheelDiameter = 0.1;

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
                new double[] { 1.6, 2.2, 3.2 },
                new double[] { 1, 82, 110 });

        fillMap(kShotFlywheelSpeedMap,
                new double[] { 1.6, 2.2, 3.2 },
                new double[] { 14.5, 15.7, 17.15 });

        fillMap(kTimeOfFlightMap,
                new double[] { 1.6, 2.2, 3.2 },
                new double[] { 0.3, 0.57, 0.7 });
    }

    public static final double kMaxShootingDist = 2.2;

    public static final double kRightLaneMultiplier = 1;
    public static final double kLeftLaneMultiplier = 3;
}
