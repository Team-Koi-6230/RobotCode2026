package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.subsystems.shooter.ballistics.BallisticsCalculator;
import frc.robot.util.AllianceFlipUtil;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.tunable.TunableManager;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public static BallisticsCalculator ballisticsCalculator;

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
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Superstate.getInstance().updateLogic();
    Logger.recordOutput("CurrentSuperstate", Superstate.getInstance().getSuperstate().toString());
    Logger.recordOutput("CurrentWantedSuperstate", Superstate.getInstance().getWantedSuperstate().toString());
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
  }

  public static boolean isInAllianceZone() {
    var robotX = AllianceFlipUtil.applyX(RobotContainer.getRobotPose().getX() - 0.3);
    return robotX < LinesVertical.allianceZone;
  }
}
