package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Superstructure.WantedState;
import frc.robot.utils.RumbleSubsystem;
import swervelib.SwerveInputStream;
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
  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IntakeArmSubsystem intakeArmSubsystem;
  private final IntakeRollerSubsytem intakeRollerSubsytem;
  private final ClimberSubsystem climberSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final SwerveSubsystem drivebase;

  private final SwerveInputStream driveAngularVelocity;
  private final SwerveInputStream driveDirectAngle;

  public RobotContainer() {

    // Controllers
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Superstructure & subsystems (lazy init)
    superstructure = Superstructure.getInstance();

    rumbleSubsystem = superstructure.getRumbleSubsystem();
    shooterSubsystem = superstructure.getShooterSubsystem();
    feederSubsystem = superstructure.getFeederSubsystem();
    intakeArmSubsystem = superstructure.getIntakeArmSubsystem();
    intakeRollerSubsytem = superstructure.getIntakeRollerSubsystem();
    climberSubsystem = superstructure.getClimberSubsystem();
    hoodSubsystem = superstructure.getHoodSubsystem();
    drivebase = superstructure.getDrivebase();

    setupAutons();

    // Input streams
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * (-1),
        () -> m_driverController.getLeftX() * (-1))
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
        .deadband(OperatorConstants.kDeadband)
        .scaleTranslation(1)
        .allianceRelativeControl(false);

    driveDirectAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(() -> m_driverController.getRightX() * -0.7,
            () -> m_driverController.getRightY() * -0.7)
        .headingWhile(true);

    configureBindings();
  }

  private void setupAutons() {
    autonChooser.setDefaultOption("get off the line", new RunCommand(() -> {
      drivebase.zeroGyro();
      drivebase.drive(new ChassisSpeeds(1, 0, 0));

    }, drivebase).withTimeout(4));

    SmartDashboard.putData("Auton/Auton Chooser", autonChooser);
  }

  private void configureBindings() {
    rumbleSubsystem.setControllers(m_driverController, m_operatorController);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    m_driverController.rightBumper().onTrue(
        Commands.runOnce(() -> superstructure.setWantedState(WantedState.PREPARING_SHOOTER), superstructure));

    m_driverController.rightBumper().onFalse(
        Commands.runOnce(() -> superstructure.setWantedState(WantedState.IDLE), superstructure));

    m_driverController.a().onTrue(
        Commands.runOnce(drivebase::zeroGyro));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
    
  }
}
