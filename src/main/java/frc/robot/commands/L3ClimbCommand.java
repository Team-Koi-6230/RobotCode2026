package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.L3ClimbJourney;

public class L3ClimbCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private L3ClimbJourney L3Journey;
    public L3ClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
         switch (L3Journey) {
      case NONE:
      climberSubsystem.setPositionGround(Constants.ClimberConstants.kL1ExtendHeight);
      break;
      case L1Open:
      climberSubsystem.setPositionHang(Constants.ClimberConstants.kL1CloseHeight);
      break;
      case L1Closed:
      climberSubsystem.setPositionHang(Constants.ClimberConstants.kL2ExtendHeight);
      break;
      case L2Open:
      climberSubsystem.setPositionHang(Constants.ClimberConstants.kL2CloseHeight);
      break;
      case L3Closed:
      climberSubsystem.setPositionHang(Constants.ClimberConstants.kL3ExtendHeight);
      break;
      default:
        break;
         }
    }

    @Override
    public void end(boolean interrupted) {
    }
}
