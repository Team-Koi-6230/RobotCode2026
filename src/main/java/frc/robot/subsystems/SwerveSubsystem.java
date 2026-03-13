package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Superstructure.WantedState;
import frc.robot.FieldConstants;
import frc.robot.utils.AllianceFlipUtil;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    public enum SwerveState {
        IDLE,
        TELEOP,
        AUTO,
        VISION_AIMING,
        AUTO_DRIVE
    }

    // #region initialization
    private final SwerveDrive swerveDrive;
    private final boolean visionDriveTest = true;
    private Vision vision;

    // Switched to Profiled PID for smoother rotation tracking
    private final ProfiledPIDController staticRotationController, movingRotationController;
    private final SimpleMotorFeedforward rotationFF;

    private SwerveState state = SwerveState.IDLE;
    private WantedState wantedState = WantedState.IDLE;

    // sim remove pre comp:
    private final FieldObject2d m_headingVisualizer;
    private final FieldObject2d m_targetVisualizer;
    private final FieldObject2d m_hubCenter;
    private final FieldObject2d m_shooterPos;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        if (RobotBase.isSimulation())
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        else
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                    Constants.SwerveDriveConstants.kMaxSpeed,
                    new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
                            Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        if (visionDriveTest) {
            setupVision();
            // Odometry thread stopped — we drive it manually in periodic()
            // so vision and odometry updates stay in sync on the main thread.
            swerveDrive.stopOdometryThread();
        }
        setupPathPlanner();

        // Feedforward for rotation
        rotationFF = new SimpleMotorFeedforward(
            Constants.SwerveDriveConstants.kSr, 
            Constants.SwerveDriveConstants.kVr, 
            Constants.SwerveDriveConstants.kAr);

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
            Constants.SwerveDriveConstants.kMaxRotVelocity,
            Constants.SwerveDriveConstants.kMaxRotAcceleration
        );

        staticRotationController = new ProfiledPIDController(
                Constants.SwerveDriveConstants.kPr,
                Constants.SwerveDriveConstants.kIr,
                Constants.SwerveDriveConstants.kDr,
                rotationConstraints);
        staticRotationController.enableContinuousInput(-Math.PI, Math.PI);

        movingRotationController = new ProfiledPIDController(
                Constants.SwerveDriveConstants.kPrMoving,
                Constants.SwerveDriveConstants.kIrMoving,
                Constants.SwerveDriveConstants.kDrMoving,
                rotationConstraints);
        movingRotationController.enableContinuousInput(-Math.PI, Math.PI);

        if (RobotBase.isSimulation()) {
            m_headingVisualizer = swerveDrive.field.getObject("Heading");
            m_targetVisualizer = swerveDrive.field.getObject("Target");
            m_hubCenter = swerveDrive.field.getObject("hub center");
            m_shooterPos = swerveDrive.field.getObject("shooterPos");
        } else {
            m_headingVisualizer = null;
            m_targetVisualizer = null;
            m_hubCenter = null;
            m_shooterPos = null;
        }
    }

    /**
     * Construct the swerve drive.
     *
     * @param driveCfg      SwerveDriveConfiguration for the swerve.
     * @param controllerCfg Swerve Controller.
     */
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg,
            SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg,
                Constants.SwerveDriveConstants.kMaxSpeed,
                new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                        Rotation2d.fromDegrees(0)));

        rotationFF = new SimpleMotorFeedforward(
            Constants.SwerveDriveConstants.kSr, 
            Constants.SwerveDriveConstants.kVr, 
            Constants.SwerveDriveConstants.kAr);

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
            Constants.SwerveDriveConstants.kMaxRotVelocity,
            Constants.SwerveDriveConstants.kMaxRotAcceleration
        );

        staticRotationController = new ProfiledPIDController(
                Constants.SwerveDriveConstants.kPr,
                Constants.SwerveDriveConstants.kIr,
                Constants.SwerveDriveConstants.kDr,
                rotationConstraints);
        staticRotationController.enableContinuousInput(-Math.PI, Math.PI);

        movingRotationController = new ProfiledPIDController(
                Constants.SwerveDriveConstants.kPrMoving,
                Constants.SwerveDriveConstants.kIrMoving,
                Constants.SwerveDriveConstants.kDrMoving,
                rotationConstraints);
        movingRotationController.enableContinuousInput(-Math.PI, Math.PI);

        if (RobotBase.isSimulation()) {
            m_headingVisualizer = swerveDrive.field.getObject("Heading");
            m_targetVisualizer = swerveDrive.field.getObject("Target");
            m_hubCenter = swerveDrive.field.getObject("hub center");
            m_shooterPos = swerveDrive.field.getObject("shooterPos");
        } else {
            m_headingVisualizer = null;
            m_targetVisualizer = null;
            m_hubCenter = null;
            m_shooterPos = null;
        }
    }

    public void setupVision() {
        vision = Vision.getInstance();
    }

    @Override
    public void periodic() {
        if (!visionDriveTest)
            return;

        // Update wheel odometry first (required since we stopped the odometry thread).
        swerveDrive.updateOdometry();

        // Vision runs EVERY loop at 50 Hz.
        //
        // Previously this was gated at 10 Hz (kVisionPeriod = 0.1 s). That was too
        // slow — the Kalman filter was working from a 100 ms stale pose. With the
        // odometry thread stopped we own the timing, so running every loop is correct
        // and costs very little (it's just a NetworkTables read + matrix multiply).
        //
        // The MT1 yaw correction fires internally in Vision on its own 0.5 s timer.
        vision.updatePoseEstimation(swerveDrive);

        // Dashboard
        SmartDashboard.putNumber("SwerveVision/HubDist_m",
                vision.getDistanceToHub());
        SmartDashboard.putNumber("SwerveVision/AimAngleDeg",
                vision.getBestAimAngle().getDegrees());
        SmartDashboard.putString("Swerve/State", state.toString());

        if (RobotBase.isSimulation()) {
            boolean isShooting = (wantedState == WantedState.PREPARING_SHOOTER
                    || wantedState == WantedState.SHOOTING
                    || wantedState == WantedState.PREPARING_SHOOTER_AND_INTAKING
                    || wantedState == WantedState.SHOOTING_AND_INTAKING)
                    && vision.isInAllianceZone();
            if (isShooting) {
                double heading = getPose().getRotation().getRadians();
                double arrowLength = 6;

                double endX = getPose().getX() + (arrowLength * Math.cos(heading));
                double endY = getPose().getY() + (arrowLength * Math.sin(heading));
                Pose2d endPoint = new Pose2d(endX, endY, getPose().getRotation());

                m_headingVisualizer.setPoses(getPose(), endPoint);

                double target = vision.getBestAimAngle().getRadians();
                arrowLength = 6;

                endX = getPose().getX() + (arrowLength * Math.cos(target));
                endY = getPose().getY() + (arrowLength * Math.sin(target));
                endPoint = new Pose2d(endX, endY, getPose().getRotation());
                m_targetVisualizer.setPoses(getPose(), endPoint);

                

                var hub = AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint);
                m_hubCenter.setPose(new Pose2d(hub.getX(), hub.getY(), new Rotation2d()));
                m_shooterPos.setPose(vision.getShooterPose());
            }
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean isReady() {
        return true;
    }

    public SwerveState getState() {
        return this.state;
    }

    public Vision getVision() {
        return this.vision;
    }

    public void setupPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedforward = true;
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotVelocity,
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics
                                            .toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    new PPHolonomicDriveController(
                            new PIDConstants(Constants.PathPlanner.kTranslationP,
                                    Constants.PathPlanner.kTranslationI,
                                    Constants.PathPlanner.kTranslationD),
                            new PIDConstants(Constants.PathPlanner.kRotationP,
                                    Constants.PathPlanner.kRotationI,
                                    Constants.PathPlanner.kRotationD)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent()
                                && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this);
        } catch (Exception e) {
            e.printStackTrace();
        }
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    public Command getAutonomousCommand(String pathName) {
        return new PathPlannerAuto(pathName);
    }
    // #endregion

    // #region drive commands
    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveDrive.getMaximumChassisAngularVelocity(),
                Units.degreesToRadians(720));
        return AutoBuilder.pathfindToPose(pose, constraints,
                edu.wpi.first.units.Units.MetersPerSecond.of(0));
    }

    /**
     * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
     * PathPlanner.
     *
     * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
     * achieve.
     * @return {@link Command} to run.
     * @throws IOException    If the PathPlanner GUI settings is invalid
     * @throws ParseException If PathPlanner GUI settings is nonexistent.
     */
    private Command driveWithSetpointGenerator(
            Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
                RobotConfig.fromGUISettings(),
                swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                        swerveDrive.getStates(),
                        DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()), () -> {
            double newTime = Timer.getFPGATimestamp();
            SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
                    prevSetpoint.get(),
                    robotRelativeChassisSpeed.get(),
                    newTime - previousTime.get());
            swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                    newSetpoint.moduleStates(),
                    newSetpoint.feedforwards().linearForces());
            prevSetpoint.set(newSetpoint);
            previousTime.set(newTime);
        });
    }

    /**
     * Drive with 254's Setpoint generator; port written by PathPlanner.
     *
     * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
     * @return Command to drive the robot using the setpoint generator.
     */
    public Command driveWithSetpointGeneratorFieldRelative(
            Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeeds.get(), getHeading()));
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();
    }

    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
                3.0, 4.0, 3.0);
    }

    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * Returns a Command that centers the modules of the SwerveDrive subsystem.
     *
     * @return a Command that centers the modules of the SwerveDrive subsystem
     */
    public Command centerModulesCommand() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    /**
     * Returns a Command that drives the swerve drive to a specific distance at a
     * given speed.
     *
     * @param distanceInMeters       the distance to drive in meters
     * @param speedInMetersPerSecond the speed at which to drive in meters per
     * second
     * @return a Command that drives the swerve drive to a specific distance at a
     * given speed
     */
    public Command driveToDistanceCommand(double distanceInMeters,
            double speedInMetersPerSecond) {
        return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
                .until(() -> swerveDrive.getPose().getTranslation()
                        .getDistance(new Translation2d(0, 0)) > distanceInMeters);
    }

    /**
     * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
     * object.
     *
     * @param kS the static gain of the feedforward
     * @param kV the velocity gain of the feedforward
     * @param kA the acceleration gain of the feedforward
     */
    public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(
                new SimpleMotorFeedforward(kS, kV, kA));
    }

    /**
     * Command to drive the robot using translative values and heading as angular
     * velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother
     * controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother
     * controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for
     * smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction. Cubed for smoother
     * controls.
     * @param translationY Translation in the Y direction. Cubed for smoother
     * controls.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {

        return run(() -> driveBasedOnState(velocity.get()));
    }

    // #endregion

    // #region drive methods
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    /**
     * Drive based on superstructure wanted state.
     *
     * When aiming, replaces the target in driveWhileAiming with the
     * lead-compensated angle from Vision. This is the key change from the
     * original — we no longer aim at a static Pose2d; we aim at the angle
     * Vision computes after lead compensation.
     */
    public void driveBasedOnState(ChassisSpeeds velocity) {
        if (Superstructure.getInstance().getIsSlowMode()) {
            velocity.omegaRadiansPerSecond = Math.min(velocity.omegaRadiansPerSecond, SwerveDriveConstants.kSlowAngularSpeedMax);
            velocity.vxMetersPerSecond = Math.min(velocity.vxMetersPerSecond, SwerveDriveConstants.kSlowSpeedMax);
            velocity.vyMetersPerSecond = Math.min(velocity.vyMetersPerSecond, SwerveDriveConstants.kSlowSpeedMax);
        }

        state = SwerveState.TELEOP;

        if (Superstructure.getInstance().isManualMode()) {
            swerveDrive.driveFieldOriented(velocity);
            return;
        }

        boolean isShooting = wantedState == WantedState.PREPARING_SHOOTER
                || wantedState == WantedState.SHOOTING
                || wantedState == WantedState.PREPARING_SHOOTER_AND_INTAKING
                || wantedState == WantedState.SHOOTING_AND_INTAKING;

        if (isShooting) {
            if (!vision.isInAllianceZone()) {
                swerveDrive.driveFieldOriented(velocity);
                return;
            }

            double dist = vision.getDistanceToHub();
            if (dist > ShooterConstants.kMaxShootingDist) {
                System.out.println("The robot is too far");
                swerveDrive.driveFieldOriented(velocity);
                return;
            }

            // Use lead-compensated aim angle from Vision.
            // driveWhileAiming(ChassisSpeeds, Rotation2d) is the new overload
            // that accepts a pre-computed heading — no static Pose2d needed.
            driveWhileAiming(velocity, vision.getBestAimAngle());
            SmartDashboard.putNumber("Swerve/target angle", vision.getBestAimAngle().getDegrees());
            SmartDashboard.putNumber("Swerve/angle", getPose().getRotation().getDegrees());
            SmartDashboard.putNumber("Swerve/error",
                    vision.getBestAimAngle().getDegrees() - getPose().getRotation().getDegrees());
            ChassisSpeeds fieldVel = getFieldVelocity();
            double speed = Math.hypot(
                    fieldVel.vxMetersPerSecond,
                    fieldVel.vyMetersPerSecond);
            SmartDashboard.putBoolean("Swerve/is moving", speed > ShooterConstants.kMeaningfullVelocity);
            return;
        }

        swerveDrive.driveFieldOriented(velocity);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }
    // #endregion

    // #region Vision Aiming

    /**
     * Drives with translation while rotating to a pre-computed heading setpoint.
     *
     * <p>
     * This is the new primary aiming overload. The heading setpoint comes from
     * {@link Vision#getBestAimAngle(SwerveDrive)}, which automatically selects
     * between static aim and lead-compensated moving aim based on robot velocity.
     *
     * <p>
     * Translation is scaled down smoothly while the heading error is large
     * (same logic as before), but the PID setpoint is now the Vision-computed
     * angle instead of a raw vector to a static Pose2d target.
     *
     * @param velocity    Field-relative driver translation + omega input.
     * @param targetAngle Pre-computed field-relative heading setpoint.
     */
    public void driveWhileAiming(ChassisSpeeds velocity, Rotation2d targetAngle) {
        state = SwerveState.VISION_AIMING;

        Translation2d hubCenter = vision.getAllianceHubCenter();
        Pose2d robotPose = getPose();

        // Decompose translation into radial (toward hub) and tangential components.
        // This preserves the tangential speed limiter logic from the original.
        Translation2d toHub = hubCenter.minus(robotPose.getTranslation());
        double distance = toHub.getNorm();

        if (distance < 1e-6) {
            swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            return;
        }

        Translation2d hubDir = toHub.div(distance);
        Translation2d desiredVel = new Translation2d(
                velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);

        double radialSpeed = desiredVel.dot(hubDir);
        Translation2d tangentialVec = desiredVel.minus(hubDir.times(radialSpeed));
        double tangentialSpeed = tangentialVec.getNorm();

        // Limit tangential speed
        if (tangentialSpeed > Constants.SwerveDriveConstants.kMaxStrafe) {
            double scale = Constants.SwerveDriveConstants.kMaxStrafe / tangentialSpeed;
            tangentialVec = tangentialVec.times(scale);
            radialSpeed *= scale;
        }

        // Limit overall speed
        Translation2d limitedVel = hubDir.times(radialSpeed).plus(tangentialVec);
        double finalSpeed = limitedVel.getNorm();
        if (finalSpeed > Constants.SwerveDriveConstants.kMaxSpeed) {
            limitedVel = limitedVel.times(Constants.SwerveDriveConstants.kMaxSpeed / finalSpeed);
        }

        // Scale translation based on heading error (don't drive much while rotating)
        Rotation2d heading = getHeading();
        double angleErrorRad = targetAngle.minus(heading).getRadians();
        double angleErrorAbs = Math.abs(angleErrorRad);

        final double kFullRotateThreshold = Math.toRadians(30);
        final double kLockedThreshold = Math.toRadians(3);

        double translationScale;
        if (angleErrorAbs > kFullRotateThreshold) {
            translationScale = 0.3;
        } else if (angleErrorAbs > kLockedThreshold) {
            translationScale = 0.3 + 0.7
                    * (kFullRotateThreshold - angleErrorAbs)
                    / (kFullRotateThreshold - kLockedThreshold);
        } else {
            translationScale = 1.0;
        }

        velocity.vxMetersPerSecond = limitedVel.getX()
                * Constants.SwerveDriveConstants.kAimingSpeedModifier * translationScale;
        velocity.vyMetersPerSecond = limitedVel.getY()
                * Constants.SwerveDriveConstants.kAimingSpeedModifier * translationScale;

        ChassisSpeeds fieldVel = getFieldVelocity();
        double speed = Math.hypot(
                fieldVel.vxMetersPerSecond,
                fieldVel.vyMetersPerSecond);

        ProfiledPIDController RotationPID;
        if (speed > ShooterConstants.kMeaningfullVelocity) {
            RotationPID = movingRotationController;
        } else {
            RotationPID = staticRotationController;
        }

        // Profiled PID handles the smooth "path" to the target angle
        double pidOutput = RotationPID.calculate(heading.getRadians(), targetAngle.getRadians());
        
        // Feedforward predicts how much voltage is needed for the profile's velocity
        double ffOutput = rotationFF.calculate(RotationPID.getSetpoint().velocity);

        velocity.omegaRadiansPerSecond = pidOutput + ffOutput;

        velocity.omegaRadiansPerSecond = MathUtil.clamp(
                velocity.omegaRadiansPerSecond,
                -Constants.SwerveDriveConstants.kMaxAngularVelocity,
                Constants.SwerveDriveConstants.kMaxAngularVelocity);

        if (angleErrorAbs < kLockedThreshold) {
            velocity.omegaRadiansPerSecond = 0.0;
            RotationPID.reset(heading.getRadians()); // Reset profile to prevent integration windup or weird stutter
        }

        SmartDashboard.putNumber("VisionAiming/HeadingDeg", heading.getDegrees());
        SmartDashboard.putNumber("VisionAiming/TargetAngleDeg", targetAngle.getDegrees());
        SmartDashboard.putNumber("VisionAiming/AngleErrorDeg",
                Math.toDegrees(angleErrorRad));
        SmartDashboard.putNumber("VisionAiming/OmegaOutput",
                velocity.omegaRadiansPerSecond);
        SmartDashboard.putNumber("VisionAiming/TranslationScale", translationScale);
        SmartDashboard.putBoolean("VisionAiming/IsAligned",
                angleErrorAbs < kLockedThreshold);

        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Legacy overload that aims at a static Pose2d target (used by auton paths
     * or any caller that still has a concrete target pose).
     *
     * Internally computes the angle to that target, then delegates to the
     * heading-setpoint overload above.
     *
     * @param velocity Field-relative driver translation + omega input.
     * @param target   Static target pose to aim at.
     */
    public void driveWhileAiming(ChassisSpeeds velocity, Pose2d target) {
        Translation2d toTarget = target.getTranslation()
                .minus(getPose().getTranslation());
        if (toTarget.getNorm() < 1e-6) {
            swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
            return;
        }
        driveWhileAiming(velocity, toTarget.getAngle());
    }

    /**
     * Whether the robot heading is within aim tolerance of the current
     * lead-compensated aim angle.
     *
     * Use this as a "ready to fire" gate combined with ShooterSubsystem.isReady().
     */
    public boolean isAimAligned() {
        if (vision == null)
            return false;
        Rotation2d aimAngle = vision.getBestAimAngle();
        Rotation2d heading = getHeading();
        double errorRad = Math.abs(aimAngle.minus(heading).getRadians());
        ChassisSpeeds fieldVel = getFieldVelocity();
        double speed = Math.hypot(
                fieldVel.vxMetersPerSecond,
                fieldVel.vyMetersPerSecond);
        if (speed < ShooterConstants.kMeaningfullVelocity) return errorRad < Constants.SwerveDriveConstants.kTargetErrorToleranceStatic;
        return errorRad < Constants.SwerveDriveConstants.kTargetErrorToleranceMoving;
    }

    /**
     * Hub-relative velocity decomposition.
     * Fixed to use the alliance-aware hub center from Vision.
     */
    public record HubRelativeVelocity(double radialSpeed, double strafeSpeed) {
    }

    public HubRelativeVelocity getHubRelativeVelocity() {
        if (vision == null)
            return new HubRelativeVelocity(0.0, 0.0);

        Translation2d toHub = vision.getAllianceHubCenter()
                .minus(swerveDrive.getPose().getTranslation());
        double distance = toHub.getNorm();

        if (distance < 1e-6)
            return new HubRelativeVelocity(0.0, 0.0);

        Translation2d hubDir = toHub.div(distance);
        ChassisSpeeds speeds = swerveDrive.getFieldVelocity();
        Translation2d robotVel = new Translation2d(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        double radialSpeed = robotVel.dot(hubDir);
        Translation2d tangential = robotVel.minus(hubDir.times(radialSpeed));

        return new HubRelativeVelocity(radialSpeed, tangential.getNorm());
    }
    // #endregion

    // #region util

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public void setIntialPose(Pose2d position) {
        swerveDrive.addVisionMeasurement(position, Timer.getFPGATimestamp());
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent()
                && alliance.get() == DriverStation.Alliance.Red;
    }

    public void zeroGyroWithAlliance() {
        zeroGyro();
        if (isRedAlliance()) {
            resetOdometry(new Pose2d(getPose().getTranslation(),
                    Rotation2d.fromDegrees(180)));
        }
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput,
            double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(
                new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(), scaledInputs.getY(),
                headingX, headingY,
                getHeading().getRadians(),
                Constants.SwerveDriveConstants.kMaxSpeed);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput,
            Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(
                new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(
                scaledInputs.getX(), scaledInputs.getY(),
                angle.getRadians(),
                getHeading().getRadians(),
                Constants.SwerveDriveConstants.kMaxSpeed);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void lock() {
        swerveDrive.lockPose();
    }

    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(
                new Pose2d(3, 3, Rotation2d.fromDegrees(65)),
                Timer.getFPGATimestamp());
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
    // #endregion
}