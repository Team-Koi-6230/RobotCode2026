package frc.robot.subsystems.shooter;

import frc.robot.util.roller.RollerConfig;

public class ShooterConstants {
    public static class Flywheel {
        public static final int kShooterSmartCurrentLimit = 80;

        public static final boolean kFollowerInverted = true;

        public static final double kUnjamVolts = -4;

        public static final double kRpmErrorTolerance = 50;

        public static final double kPSim = 0.2;
        public static final double kISim = 0;
        public static final double kDSim = 0;
        public static final double kSSim = 0.01;
        public static final double kVSim = 0.01651; 
        public static final double kASim = 0;

        public static final double kP = 0.00008;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.01;
        public static final double kV = 0.0018;
        public static final double kA = 0;
    }

    public static class Hood {
        public static final double kHoodMaxAngle = 180;

        public static final double kNonShootingAngle = 180;
    }

    public static class Roller {
        public static final double kRollerGearRatio = 4;
        public static final int kRollerSmartCurrentLimit = 60;
        public static final RollerConfig.RollerMotor kRollerMotor = RollerConfig.RollerMotor.NEO;

        public static final double kFeedVolts = 12;
        public static final double kUnjamVolts = -6;
    }
}
