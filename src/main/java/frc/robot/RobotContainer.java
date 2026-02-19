package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.RumbleSubsystem;

import swervelib.SwerveInputStream;

public class RobotContainer {
        private final CommandXboxController driverController;
        private final CommandXboxController operatorController;

        private final Superstructure superstructure;
        private final SwerveSubsystem drivebase;
        private final RumbleSubsystem rumbleSubsystem;

        private final SwerveInputStream driveAngularVelocity;

        private final SendableChooser<Command> autonChooser = new SendableChooser<>();

        public RobotContainer() {
                driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
                operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

                superstructure = Superstructure.getInstance();
                drivebase = superstructure.getDrivebase();
                rumbleSubsystem = superstructure.getRumbleSubsystem();

                driveAngularVelocity = SwerveInputStream.of(
                                drivebase.getSwerveDrive(),
                                () -> -driverController.getLeftY(),
                                () -> -driverController.getLeftX())
                                .withControllerRotationAxis(() -> driverController.getRightX())
                                .deadband(OperatorConstants.kDeadband)
                                .scaleTranslation(1.0)
                                .allianceRelativeControl(true);

                registerAutonCommands();
                setupAutons();
                configureBindings();
        }

        private void registerAutonCommands() {
                NamedCommands.registerCommand("Intake", superstructure.setINTAKINGstate());
                NamedCommands.registerCommand("Prepare shooting", superstructure.setPREPARING_SHOOTERstate());
                NamedCommands.registerCommand("Shoot", superstructure.setSHOOTINGstate());
                NamedCommands.registerCommand("L1 Climb", superstructure.setL1_CLIMBstate());
                NamedCommands.registerCommand("Idle", superstructure.setIDLEstate());
                NamedCommands.registerCommand("Home", superstructure.setHOMEstate());
        }

        private void setupAutons() {
                autonChooser.setDefaultOption(
                                "Get Off Line",
                                new RunCommand(
                                                () -> {
                                                        drivebase.zeroGyro();
                                                        drivebase.drive(new ChassisSpeeds(1.0, 0.0, 0.0));
                                                },
                                                drivebase)
                                                .withTimeout(4.0));

                SmartDashboard.putData("Auton/Chooser", autonChooser);
        }

        private void configureBindings() {
                BooleanSupplier superstateMode = superstructure::isSuperstateMode;
                BooleanSupplier manualMode = superstructure::isManualMode;

                rumbleSubsystem.setControllers(driverController, operatorController);

                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
                superstructure.setDefaultCommand(superstructure.setIDLEstate());

                driverController.rightBumper()
                                .and(superstateMode)
                                .and(() -> !driverController.leftTrigger().getAsBoolean())
                                .whileTrue(superstructure.setPREPARING_SHOOTERstate());

                driverController.rightTrigger()
                                .and(superstateMode)
                                .and(() -> !driverController.leftTrigger().getAsBoolean())
                                .whileTrue(superstructure.setSHOOTINGstate());

                driverController.leftTrigger()
                                .and(superstateMode)
                                .and(() -> !(driverController.rightBumper().getAsBoolean()
                                                && driverController.rightTrigger().getAsBoolean()))
                                .whileTrue(superstructure.setINTAKINGstate());

                driverController.leftTrigger()
                                .and(superstateMode)
                                .and(driverController.rightBumper())
                                .whileTrue(superstructure.setPREPARING_SHOOTER_AND_INTAKINGshooting());

                driverController.leftTrigger()
                                .and(superstateMode)
                                .and(driverController.rightBumper())
                                .whileTrue(superstructure.setSHOOTING_AND_INTAKINGshooting());

                driverController.b()
                                .and(superstateMode)
                                .whileTrue(superstructure.setL3_CLIMBstate());

                driverController.x()
                                .and(superstateMode)
                                .whileTrue(superstructure.setHOMEstate());

                driverController.povRight()
                                .and(manualMode)
                                .onTrue(new InstantCommand(drivebase::zeroGyro));

                // driverController.x()
                // .and(manualMode)
                // .onTrue(superstructure.getClimberSubsystem().extendClimberHangCommand());

                // driverController.y()
                // .and(manualMode)
                // .onTrue(superstructure.getClimberSubsystem().closeClimberGroundCommand());

                // driverController.b()
                // .and(manualMode)
                // .onTrue(superstructure.getClimberSubsystem().extendClimberGroundCommand());

                // driverController.a()
                // .and(manualMode)
                // .onTrue(superstructure.getClimberSubsystem().extendClimberHangCommand());

                driverController.leftTrigger()
                                .onTrue(instantCommand(
                                                superstructure.getIntakeArmSubsystem()::OpenArm))
                                .onFalse(instantCommand(
                                                superstructure.getIntakeArmSubsystem()::CloseArm));

                driverController.rightBumper()
                                .and(manualMode)
                                .onTrue(superstructure
                                                .getIntakeRollerSubsystem()
                                                .rollerSpinCommand(Constants.IntakeRollerConstants.kIntakePower))
                                .onFalse(superstructure
                                                .getIntakeRollerSubsystem()
                                                .rollerSpinCommand(0.0));

                driverController.rightTrigger()
                                .and(manualMode)
                                .onTrue(superstructure
                                                .getShooterSubsystem()
                                                .setVelocityCommand(4000))
                                .onFalse(superstructure
                                                .getShooterSubsystem()
                                                .setVelocityCommand(0));

                driverController.leftBumper()
                                .and(manualMode)
                                .onTrue(superstructure
                                                .getFeederSubsystem()
                                                .feederSpinCommand(12))
                                .onFalse(superstructure
                                                .getFeederSubsystem()
                                                .feederSpinCommand(0));

                operatorController.a()
                                .and(manualMode)
                                .whileTrue(superstructure
                                                .getHoodSubsystem()
                                                .joystickHoodControl(() -> -operatorController.getRightY()));

                driverController.povLeft()
                                .onTrue(superstructure.toggleControlState());

                operatorController.y().onTrue(superstructure.getHoodSubsystem().setHoodAngleCommand(45))
                                .onFalse(superstructure.getHoodSubsystem().setHoodAngleCommand(0));
        }

        public Command getAutonomousCommand() {
                return autonChooser.getSelected();
        }

        private static InstantCommand instantCommand(Runnable runnable) {
                return new InstantCommand(runnable) {
                        @Override
                        public boolean runsWhenDisabled() {
                                return true;
                        }
                };
        }
}