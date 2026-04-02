package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
        public static final double maxSpeedMetersPerSec = 4.8;
        public static final double odometryFrequency = 100.0;
        public static final double trackWidth = Units.inchesToMeters(26.5);
        public static final double wheelBase = Units.inchesToMeters(26.5);
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
                        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        public static final class SwerveModuleConfig {
                public final int driveCanId;
                public final int turnCanId;
                public final int cancoderId;
                public final Rotation2d angleOffset;
                public final boolean driveInverted;
                public final boolean turnInverted;
                public final boolean cancoderInverted;

                public SwerveModuleConfig(int driveCanId, int turnCanId, int cancoderId,
                                Rotation2d angleOffset, boolean driveInverted,
                                boolean turnInverted, boolean cancoderInverted) {
                        this.driveCanId = driveCanId;
                        this.turnCanId = turnCanId;
                        this.cancoderId = cancoderId;
                        this.angleOffset = angleOffset;
                        this.driveInverted = driveInverted;
                        this.turnInverted = turnInverted;
                        this.cancoderInverted = cancoderInverted;
                }
        }

        public static final SwerveModuleConfig[] moduleConfigs = new SwerveModuleConfig[] {
                        // Front Left (Index 0)
                        new SwerveModuleConfig(1, 2, 9, new Rotation2d(0.0), false, false, false),
                        // Front Right (Index 1)
                        new SwerveModuleConfig(5, 6, 11, new Rotation2d(0.0), false, false, false),
                        // Back Left (Index 2)
                        new SwerveModuleConfig(3, 4, 10, new Rotation2d(0.0), false, false, false),
                        // Back Right (Index 3)
                        new SwerveModuleConfig(7, 8, 12, new Rotation2d(0.0), false, false, false)
        };

        public static SwerveModuleConfig getModuleConfig(int index) {
                return moduleConfigs[index];
        }

        public static final int driveMotorCurrentLimit = 60;
        public static final double wheelRadiusMeters = Units.inchesToMeters(2.0);
        public static final double driveMotorReduction = 6.75;
        public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

        public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction;
        public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction;

        public static final double driveKp = 0.0;
        public static final double driveKd = 0.0;
        public static final double driveKs = 0.0;
        public static final double driveKv = 0.1;
        public static final double driveSimP = 0.05;
        public static final double driveSimD = 0.0;
        public static final double driveSimKs = 0.0;
        public static final double driveSimKv = 0.0789;

        public static final int turnMotorCurrentLimit = 20;
        public static final double turnMotorReduction = 12.8;
        public static final DCMotor turnGearbox = DCMotor.getNeoVortex(1);

        public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
        public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec
        public static final double kCANcoderFactor = turnEncoderPositionFactor;

        public static final double turnKp = 2.0;
        public static final double turnKd = 0.0;
        public static final double turnSimP = 8.0;
        public static final double turnSimD = 0.0;
        public static final double turnPIDMinInput = 0;
        public static final double turnPIDMaxInput = 2 * Math.PI;

        public static final double kPaiming = 7.5, kIaiming = 0, kDaiming = 0.75;
        public static final Rotation2d kAimingTolerance = Rotation2d.fromDegrees(1);

        public static final double[] kRoundedOrientations = new double[] { 0, Math.PI / 2, Math.PI,
                        Math.PI + (Math.PI / 2) };

        public static final double kFalloffPercent = 0.075;
        public static final int kFalloffExponent = 10;

        public static final double robotMassKg = 45;
        public static final double robotMOI = 6.883;
        public static final double wheelCOF = 1.2;
        public static final RobotConfig ppConfig = new RobotConfig(
                        robotMassKg,
                        robotMOI,
                        new ModuleConfig(
                                        wheelRadiusMeters,
                                        maxSpeedMetersPerSec,
                                        wheelCOF,
                                        driveGearbox.withReduction(driveMotorReduction),
                                        driveMotorCurrentLimit,
                                        1),
                        moduleTranslations);
}