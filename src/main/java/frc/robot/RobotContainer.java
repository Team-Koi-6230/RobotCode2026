package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.utils.KoiController;
import team6230.koiupstream.utils.SwerveInputStream;

public class RobotContainer {

        private Superstate superstate = Superstate.getInstance();
        private static KoiController driverController = new KoiController(0, 0.08, 4, 5);

        private final LoggedDashboardChooser<Command> autoChooser;

        private Trigger IntakeButton = driverController.leftTrigger();
        private Trigger HomeButton = driverController.a();
        private Trigger PreparingShooterButton = driverController.rightBumper();
        private Trigger ShootingButton = driverController.b();
        private Trigger UnjamButton = driverController.povUp();

        private boolean isShooting = false;

        private static SwerveInputStream swerveInputStream = new SwerveInputStream(driverController::getSwerveDrive,
                        driverController::getSwerveStrafe, driverController::getSwerveTurn);

        private static Drive drive = new Drive(swerveInputStream);

        @SuppressWarnings("unused")
        private Intake intake = new Intake();
        private Shooter shooter = new Shooter();
        @SuppressWarnings("unused")
        private Vision vision = new Vision();

        public RobotContainer() {

                autoChooser = new LoggedDashboardChooser<>("Auto Choices",
                                AutoBuilder.buildAutoChooser());

                autoChooser.addDefaultOption("resetOdometry", Commands.runOnce(
                                () -> {
                                        var isRedAlliance = DriverStation.getAlliance()
                                                        .orElse(Alliance.Blue) == Alliance.Red;
                                        drive.setPose(
                                                        new Pose2d(drive.getPose().getTranslation(),
                                                                        new Rotation2d(isRedAlliance ? Math.PI : 0)));
                                },
                                drive));

                configureBindings();
        }

        private void configureBindings() {
                superstate.setDefaultWantedState(RobotState.IDLE);

                IntakeButton
                                .and(PreparingShooterButton.negate()).and(ShootingButton.negate())
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.INTAKING));

                PreparingShooterButton
                                .and(ShootingButton.negate()).and(IntakeButton.negate())
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.PREPARING_SHOOTER));

                PreparingShooterButton
                                .and(ShootingButton.negate()).and(IntakeButton)
                                .whileTrue(superstate
                                                .setWantedSuperstateCommand(RobotState.PREPARING_SHOOTER_AND_INTAKING));

                ShootingButton
                                .and(IntakeButton.negate())
                                .and(this::isReadyToShoot)
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.SHOOTING));

                ShootingButton
                                .and(IntakeButton.negate())
                                .and(this::isUnpreparedToShoot)
                                .and(() -> isShooting)
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.SHOOTING_RECOVERY));
                ShootingButton
                                .and(IntakeButton.negate())
                                .and(this::isUnpreparedToShoot)
                                .and(() -> !isShooting)
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.PREPARING_SHOOTER));

                ShootingButton
                                .and(IntakeButton)
                                .and(this::isReadyToShoot)
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.SHOOTING_AND_INTAKING));

                ShootingButton
                                .and(IntakeButton)
                                .and(this::isUnpreparedToShoot)
                                .whileTrue(superstate
                                                .setWantedSuperstateCommand(RobotState.PREPARING_SHOOTER_AND_INTAKING));

                UnjamButton
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.UNJAM));

                HomeButton
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.HOME));

                driverController.leftBumper().onTrue(Commands.runOnce(() -> {
                        drive.toggleShouldRoundOrientation();
                }));

                superstate.subscribeToStateMachine((s) -> {
                        if (s.ordinal() == RobotState.SHOOTING.ordinal()
                                        || s.ordinal() == RobotState.SHOOTING_RECOVERY.ordinal()) {
                                isShooting = true;
                        } else {
                                isShooting = false;
                        }
                }, () -> true);
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public static Pose2d getRobotPose() {
                return drive.getPose();
        }

        public static ChassisSpeeds getRobotVelocity() {
                return drive.getChassisSpeeds();
        }

        public static void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
                        Matrix<N3, N1> visionMeasurementStdDevs) {
                drive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        }

        private boolean isReadyToShoot() {
                return drive.isInAimTolerance(getRobotPose().getRotation().getRadians(),
                                Robot.ballisticsCalculator.getShootingRobotAngle().getRadians())
                                && shooter.isFlywheelInTolerance()
                                && Math.abs(getRobotVelocity().vxMetersPerSecond) < Constants.robotMeaningfulVxyMetersPerSecond
                                && Math.abs(getRobotVelocity().vyMetersPerSecond) < Constants.robotMeaningfulVxyMetersPerSecond
                                && Math.abs(getRobotVelocity().omegaRadiansPerSecond) < Constants.robotMeaningfulOmegaRadiansPerSecond;
        }

        private boolean isUnpreparedToShoot() {
                return !isReadyToShoot();
        }
}