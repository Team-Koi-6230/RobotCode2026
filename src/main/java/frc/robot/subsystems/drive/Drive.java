package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.driveBaseRadius;
import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;
import static frc.robot.subsystems.drive.DriveConstants.ppConfig;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.MathHelper;
import team6230.koiupstream.subsystems.UpstreamDrivebase;
import team6230.koiupstream.utils.SwerveInputStream;

public class Drive extends UpstreamDrivebase<RobotState> {
  static final Lock odometryLock = new ReentrantLock();
  private GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private PIDController _aimingPID;
  @AutoLogOutput
  private boolean _shouldRoundOrientation = false;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, Pose2d.kZero);

  public Drive(SwerveInputStream inputStream) {
    super(inputStream);
    getIO();
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    SparkOdometryThread.getInstance().start();

    _aimingPID = new PIDController(DriveConstants.kPaiming, DriveConstants.kIaiming, DriveConstants.kDaiming);
    _aimingPID.enableContinuousInput(-Math.PI, Math.PI);

    registerDrives();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  private void registerDrives() {
    registerDefaultDrive(this::defaultDrive);
    registerDriveMode(RobotState.PREPARING_SHOOTER, this::shootingDrive);
    registerDriveMode(RobotState.SHOOTING, this::shootingDrive);
  }

  private ChassisSpeeds defaultDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(
        xSupplier.getAsDouble(), ySupplier.getAsDouble());

    ChassisSpeeds measured = getChassisSpeeds();
    double currentSpeed = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
    double speedRatio = Math.min(currentSpeed / getMaxLinearSpeedMetersPerSec(), 1.0);
    double falloff = Math.pow(
        1.0 - speedRatio * DriveConstants.kFalloffPercent,
        DriveConstants.kFalloffExponent);
    linearVelocity = linearVelocity.times(falloff);

    Logger.recordOutput("Drive/FalloffMultiplier", falloff);
    Logger.recordOutput("Drive/SpeedRatio", speedRatio);

    double omega = 0;

    if (!_shouldRoundOrientation) {
      omega = omegaSupplier.getAsDouble();
      omega *= this.getMaxAngularSpeedRadPerSec();
    } else {
      double currentAngle = getPose().getRotation().getRadians();
      double wantedAngle = MathHelper.getClosestRadian(currentAngle, DriveConstants.kRoundedOrientations);

      omega = _aimingPID.calculate(currentAngle, wantedAngle);

      Logger.recordOutput("Drive/currentAngle", currentAngle);
      Logger.recordOutput("Drive/wantedAngle", wantedAngle);

      if (isInAimTolerance(currentAngle, wantedAngle)) {
        _shouldRoundOrientation = false;
      }
    }

    return convertFieldRelativeSpeedsToRobotRelative(new ChassisSpeeds(
        linearVelocity.getX() * this.getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * this.getMaxLinearSpeedMetersPerSec(),
        omega));
  }

  private boolean isInAimTolerance(double currentAngle, double wantedAngle) {
    return Math.abs(MathUtil.angleModulus(currentAngle - wantedAngle)) < DriveConstants.kAimingTolerance.getRadians();
  }

  private ChassisSpeeds shootingDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks(
        xSupplier.getAsDouble(), ySupplier.getAsDouble());
    Rotation2d aimingAngle = Robot.ballisticsCalculator.getShootingRobotAngle();

    double omega = _aimingPID.calculate(getRotation().getRadians(), aimingAngle.getRadians());

    return convertFieldRelativeSpeedsToRobotRelative(new ChassisSpeeds(
        linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
        linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
        omega));
  }

  private ChassisSpeeds convertFieldRelativeSpeedsToRobotRelative(ChassisSpeeds speeds) {
    boolean isFlipped = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        speeds,
        isFlipped
            ? this.getRotation().plus(new Rotation2d(Math.PI))
            : this.getRotation());
  }

  @Override
  protected void updateInputs() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  /**
   * Converts joystick x/y axes into a unit-circle-clamped translation vector.
   *
   * Input shaping is intentionally NOT done here — KoiController handles that
   * (cubic curve + deadband + slew rate limiting) before the value arrives.
   * Squaring the magnitude here in addition to KoiController's cubic would
   * produce a combined x^6 curve where nearly all input below 50% stick throw
   * is thrown away.
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = Math.hypot(-x, -y);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Clamp to unit circle — no additional shaping
    linearMagnitude = Math.min(linearMagnitude, 1.0);

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public void toggleShouldRoundOrientation() {
    _shouldRoundOrientation = !_shouldRoundOrientation;
  }

  private void getIO() {
    switch (Constants.currentMode) {
      case REAL:
        gyroIO = new GyroIO() {
        };
        for (int i = 0; i < modules.length; ++i) {
          modules[i] = new Module(new ModuleIOSpark(i), i);
        }
        break;

      case SIM:
        gyroIO = new GyroIO() {
        };
        for (int i = 0; i < modules.length; ++i) {
          modules[i] = new Module(new ModuleIOSim(), i);
        }
        break;

      default:
        gyroIO = new GyroIO() {
        };
        for (int i = 0; i < modules.length; ++i) {
          modules[i] = new Module(new ModuleIO() {
          }, i);
        }
        break;
    }
  }
}