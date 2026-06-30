package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.utils.KoiController;
import team6230.koiupstream.utils.SwerveInputStream;

public class RobotContainer {

        private Superstate superstate = Superstate.getInstance();
        private static KoiController driverController = new KoiController(0, 0.15, 8, 5);
        private CommandXboxController op = new CommandXboxController(1);
        private final LoggedDashboardChooser<Command> autoChooser;

        private Trigger IntakeButton = driverController.leftTrigger();
        private Trigger HomeButton = driverController.a();
        private Trigger PreparingShooterButton = driverController.rightBumper();
        private Trigger ShootingButton = driverController.rightTrigger();
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
        @SuppressWarnings("unused")
        private LED led = new LED();

        public RobotContainer() {
                NamedCommands.registerCommand("Shoot", Commands.run(() -> {
                        if (superstate.isCurrent(RobotState.SHOOTING)) {
                                superstate.setWantedSuperstate(RobotState.SHOOTING);
                        } else {
                                superstate.setWantedSuperstate(RobotState.PRESHOOTING);
                        }
                }, superstate));
                NamedCommands.registerCommand("Intake", superstate.setWantedSuperstateCommand(RobotState.INTAKING));
                NamedCommands.registerCommand("Idle", superstate.setWantedSuperstateCommand(RobotState.IDLE));

                autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

                autoChooser.addOption("safe middle attack like bron (right)",
                                new PathPlannerAuto("safe middle attack like bron", true));

                autoChooser.addOption("Step back three from the center (backup)",
                                Commands.runOnce(() -> {
                                        resetGyro();
                                }, drive)
                                                .andThen(
                                                                Commands.run(() -> drive.runVelocity(
                                                                                new ChassisSpeeds(-1.0, 0, 0)), drive)
                                                                                .withTimeout(1.5))
                                                .andThen(() -> drive.runVelocity(new ChassisSpeeds(0, 0, 0)))
                                                .andThen(
                                                                Commands.run(() -> {
                                                                        if (superstate.isCurrent(RobotState.SHOOTING)) {
                                                                                superstate.setWantedSuperstate(
                                                                                                RobotState.SHOOTING);
                                                                        } else {
                                                                                superstate.setWantedSuperstate(
                                                                                                RobotState.PRESHOOTING);
                                                                        }
                                                                }, superstate))
                                                .andThen(superstate.setWantedSuperstateCommand(RobotState.IDLE)));

                autoChooser.addDefaultOption("resetOdometry", Commands.runOnce(
                                () -> {
                                        resetGyro();
                                },
                                drive));

                configureBindings();
        }

        private void configureBindings() {
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
                                .and(() -> !superstate.isCurrent(RobotState.SHOOTING))
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.PRESHOOTING));

                ShootingButton
                                .and(IntakeButton.negate())
                                .and(() -> superstate.isCurrent(RobotState.SHOOTING))

                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.SHOOTING));

                UnjamButton
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.UNJAM));

                HomeButton
                                .whileTrue(superstate.setWantedSuperstateCommand(RobotState.HOME));

                driverController.povDown().whileTrue(Commands.runOnce(() -> resetGyro(), drive));

                driverController.leftBumper().onTrue(Commands.runOnce(() -> {
                        drive.toggleShouldRoundOrientation();
                }));

                driverController.y().onTrue(Commands.runOnce(() -> activateDefensiveSwerve()));

                /*
                 * op.a().whileTrue(intake.intakeManual());
                 * op.b().whileTrue(intake.cancel());
                 */
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

        public static boolean isRobotInAimTolerance(double currentAngle, double wantedAngle) {
                return drive.isInAimTolerance(currentAngle, wantedAngle);
        }

        public static void activateDefensiveSwerve() {
                drive.toggleXLock();
        }

        public static void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
                        Matrix<N3, N1> visionMeasurementStdDevs) {
                drive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        }

        public static void resetGyro() {
                var isRedAlliance = DriverStation.getAlliance()
                                .orElse(Alliance.Blue) == Alliance.Red;
                drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(),
                                                new Rotation2d(isRedAlliance ? Math.PI : 0)));
        }
}