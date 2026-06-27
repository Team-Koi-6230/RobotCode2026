package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.shooter.ballistics.BallisticsCalculator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.rebuilt.ShiftUtil;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.tunable.Tunable;
import team6230.koiupstream.tunable.TunableManager;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static BallisticsCalculator ballisticsCalculator;

  private final Field2d m_field = new Field2d();

  @Tunable
  public static boolean isShowcaseMode = false;

  public Robot() {
    Logger.recordMetadata("ProjectName", "Team Koi Robot code");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
      DataLogManager.start();
      URCL.start();
    } else {
      Logger.addDataReceiver(new NT4Publisher());
      // Logger.addDataReceiver(new WPILOGWriter(""));
    }

    Logger.start();

    Superstate.getInstance().setSuperstateSet(RobotState.IDLE);
    TunableManager.tuningModeEnabled = Constants.tuningMode;

    ballisticsCalculator = new BallisticsCalculator();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Superstate.getInstance().updateLogic();
    Logger.recordOutput("CurrentSuperstate", Superstate.getInstance().getSuperstate().toString());
    Logger.recordOutput("CurrentWantedSuperstate", Superstate.getInstance().getWantedSuperstate().toString());

    m_field.setRobotPose(RobotContainer.getRobotPose());
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Superstate.getInstance().setDefaultWantedState(RobotState.IDLE);
    CommandScheduler.getInstance().clearComposedCommands();
    ShiftUtil.initialize();
  }

  @Override
  public void teleopPeriodic() {
    ShiftUtil.publishShiftInfo();
  }

  @Override
  public void disabledPeriodic() {
    updateAlliance();
    SmartDashboard.putBoolean(
        "Shift/Alliance Win Override",
        ShiftUtil.getAllianceWinOverride().orElse(false));
  }

  public void updateAlliance() {
    if (Robot.isSimulation()) {
      Constants.isRedAlliance = DriverStationSim.getAllianceStationId() == AllianceStationID.Red1
          || DriverStationSim.getAllianceStationId() == AllianceStationID.Red2
          || DriverStationSim.getAllianceStationId() == AllianceStationID.Red3;
    } else {
      DriverStation.getAlliance()
          .ifPresentOrElse(
              alliance -> Constants.isRedAlliance = alliance == DriverStation.Alliance.Red, () -> {
                SmartDashboard.putNumber("Last unable to set alliance", Timer.getFPGATimestamp());
              });
    }
  }

  public static boolean isInAllianceZone() {
    var robotX = AllianceFlipUtil.applyX(RobotContainer.getRobotPose().getX() - 0.3);
    return robotX < LinesVertical.allianceZone;
  }
}