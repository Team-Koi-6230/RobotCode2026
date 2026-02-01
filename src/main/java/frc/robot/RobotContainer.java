package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.RumbleSubsystem;
import swervelib.SwerveInputStream;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final Superstructure superstructure;

  private final RumbleSubsystem rumbleSubsystem;
  private final SwerveSubsystem drivebase;

  private final SwerveInputStream driveAngularVelocity;

  public RobotContainer() {

    // Controllers
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Superstructure & subsystems (lazy init)
    superstructure = Superstructure.getInstance();
    drivebase = superstructure.getDrivebase();
    rumbleSubsystem = superstructure.getRumbleSubsystem();

    setupAutons();

    // Input streams
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * (-1),
        () -> m_driverController.getLeftX() * (-1))
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
        .deadband(OperatorConstants.kDeadband)
        .scaleTranslation(1)
        .allianceRelativeControl(true);

    configureBindings();
  }

  private void setupAutons() {
    setupAutonCommands();
    autonChooser.setDefaultOption("get off the line", new RunCommand(() -> {
      drivebase.zeroGyro();
      drivebase.drive(new ChassisSpeeds(1, 0, 0));

    }, drivebase).withTimeout(4));

    autonChooser.addOption("Depo tIntake Climb", drivebase.getAutonomousCommand("DepotIntakeClimb"));

    SmartDashboard.putData("Auton/Auton Chooser", autonChooser);
  }

  private void setupAutonCommands() {
    NamedCommands.registerCommand("Intake", superstructure.setINTAKINGstate());
    NamedCommands.registerCommand("Prepare shooting", superstructure.setPREPARING_SHOOTERstate());
    NamedCommands.registerCommand("Shoot", superstructure.setSHOOTINGstate());
    NamedCommands.registerCommand("L1 Climb", superstructure.setL1_CLIMBstate());
    NamedCommands.registerCommand("Idle", superstructure.setIDLEstate());
    NamedCommands.registerCommand("Home", superstructure.setHOMEstate());
  }

  private void configureBindings() {
    BooleanSupplier isSuperstate = () -> superstructure.isSuperstateMode();
    BooleanSupplier isManualMode = () -> superstructure.isManualMode();

    rumbleSubsystem.setControllers(m_driverController, m_operatorController);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    superstructure.setDefaultCommand(superstructure.setIDLEstate());

    m_driverController.a()
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    m_driverController.rightBumper()
        .and(isSuperstate)
        .whileTrue(superstructure.setPREPARING_SHOOTERstate());

    m_driverController.rightTrigger()
        .and(isSuperstate)
        .whileTrue(superstructure.setSHOOTINGstate());

    m_driverController.leftTrigger()
        .and(isSuperstate)
        .whileTrue(superstructure.setINTAKINGstate());

    m_driverController.b()
        .and(isSuperstate)
        .whileTrue(superstructure.setL3_CLIMBstate());

    m_driverController.x()
        .and(isSuperstate)
        .whileTrue(superstructure.setHOMEstate());

    m_driverController.povLeft()
        .onTrue(superstructure.toggleControlState());
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();

  }
}
