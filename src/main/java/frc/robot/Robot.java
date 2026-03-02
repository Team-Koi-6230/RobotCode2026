package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  NetworkTableEntry xEntry = NetworkTableInstance.getDefault()
      .getTable("Pose/InitialPose").getEntry("X");
  NetworkTableEntry yEntry = NetworkTableInstance.getDefault()
      .getTable("Pose/InitialPose").getEntry("Y");
  NetworkTableEntry rotEntry = NetworkTableInstance.getDefault()
      .getTable("Pose/InitialPose").getEntry("Rotation");

  private final SendableChooser<Pose2d> intialPosePicker = new SendableChooser<>();
  private Pose2d lastPose2d;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    poseIntialization();
  }

  private void poseIntialization() {
    // we will default to the center
    intialPosePicker.setDefaultOption("Center Blue", FieldConstants.centerStartBlue);
    lastPose2d = FieldConstants.centerStartBlue;
    intialPosePicker.addOption("Left Blue", FieldConstants.leftStartBlue);
    intialPosePicker.addOption("Right Blue", FieldConstants.rightStartBlue);
    intialPosePicker.addOption("Center Red", FieldConstants.centerStartRed);
    intialPosePicker.addOption("Left Red", FieldConstants.leftStartRed);
    intialPosePicker.addOption("Right Red", FieldConstants.rightStartRed);

    xEntry.setDouble(FieldConstants.centerStartBlue.getX());
    yEntry.setDouble(FieldConstants.centerStartBlue.getY());
    rotEntry.setDouble(FieldConstants.centerStartBlue.getRotation().getDegrees());

    SmartDashboard.putData("Intial position picker", intialPosePicker);
  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    DataLogManager.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Timer", (int) DriverStation.getMatchTime());

    if (DriverStation.isDisabled()) {
      var selected = intialPosePicker.getSelected();

      if (selected != null && !lastPose2d.equals(selected)) {
        xEntry.setDouble(selected.getX());
        yEntry.setDouble(selected.getY());
        rotEntry.setDouble(selected.getRotation().getDegrees());

        Superstructure.getInstance().getDrivebase().resetOdometry(selected);

        lastPose2d = selected;
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
