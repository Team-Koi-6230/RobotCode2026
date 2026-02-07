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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
    //#region Superstate Mode
    

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
    //#endregion

    //#region Manual Mode
    m_driverController.povRight()
        .and(isManualMode)
        .onTrue(Commands.runOnce(drivebase::zeroGyro));

    m_driverController.x()
        .and(isManualMode)
        .onTrue(superstructure.getClimberSubsystem().extendClimberHangCommand());
    
    m_driverController.y()
        .and(isManualMode)
        .onTrue(superstructure.getClimberSubsystem().closeClimberGroundCommand());

    m_driverController.b()
        .and(isManualMode)
        .onTrue(superstructure.getClimberSubsystem().extendClimberGroundCommand());

    m_driverController.a()
        .and(isManualMode)
        .onTrue(superstructure.getClimberSubsystem().extendClimberHangCommand());

    m_operatorController.leftTrigger()
        .and(isManualMode)
        .onTrue(Commands.runOnce(() -> superstructure.getIntakeArmSubsystem().OpenArm(), superstructure.getIntakeArmSubsystem()))
        .onFalse(Commands.runOnce(() -> superstructure.getIntakeArmSubsystem().CloseArm(), superstructure.getIntakeArmSubsystem()));
        
    m_operatorController.rightBumper()
        .and(isManualMode)
        .onTrue(superstructure.getIntakeRollerSubsystem().rollerSpinCommand(Constants.IntakeRollerConstants.kIntakePower))
        .onFalse(superstructure.getIntakeRollerSubsystem().rollerSpinCommand(0));

    m_operatorController.a()
        .and(isManualMode)
        .whileTrue(superstructure.getHoodSubsystem().joystickHoodControl(() -> -m_operatorController.getRightY()));

    m_operatorController.b()
        .and(isManualMode)
        .whileTrue(superstructure.getShooterSubsystem().joystickShooterControl(() -> -m_operatorController.getLeftY()));
    //#endregion
    m_driverController.povLeft()
        .onTrue(superstructure.toggleControlState());
  }

public Command getIntakeArmSysIdQuasistatic(SysIdRoutine.Direction direction) {
  return superstructure.getIntakeArmSubsystem().sysIdQuasistatic(direction)
    .until(() -> superstructure.getIntakeArmSubsystem().getAngle() > Constants.IntakeArmConstants.kMaxPosition ||
                 superstructure.getIntakeArmSubsystem().getAngle() < Constants.IntakeArmConstants.kMinPosition);
}

public Command getIntakeArmSysIdDynamic(SysIdRoutine.Direction direction) {
  return superstructure.getIntakeArmSubsystem().sysIdDynamic(direction)
    .until(() -> superstructure.getIntakeArmSubsystem().getAngle() > Constants.IntakeArmConstants.kMaxPosition ||
                 superstructure.getIntakeArmSubsystem().getAngle() < Constants.IntakeArmConstants.kMinPosition);
}

public Command getClimberSysIdQuasistatic(SysIdRoutine.Direction direction) {
  return superstructure.getClimberSubsystem().sysIdQuasistatic(direction)
    .until(() -> superstructure.getClimberSubsystem().getHeight() > Constants.ClimberConstants.kMaxPosition ||
                 superstructure.getClimberSubsystem().getHeight() < Constants.ClimberConstants.kMinPosition);
}

public Command getClimberSysIdDynamic(SysIdRoutine.Direction direction) {
  return superstructure.getClimberSubsystem().sysIdDynamic(direction)
    .until(() -> superstructure.getClimberSubsystem().getHeight() > Constants.ClimberConstants.kMaxPosition ||
                 superstructure.getClimberSubsystem().getHeight() < Constants.ClimberConstants.kMinPosition);
}

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();

  }
}
