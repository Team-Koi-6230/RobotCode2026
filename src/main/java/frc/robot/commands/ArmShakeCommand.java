package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;

/* You're allowed to refer to this command as the thug shake, please do */
public class ArmShakeCommand extends Command {
    private final IntakeArmSubsystem intakeArmSubsystem;
    

    public ArmShakeCommand(IntakeArmSubsystem intakeArmSubsystem) {
        this.intakeArmSubsystem = intakeArmSubsystem;
        addRequirements(intakeArmSubsystem);
    
    }

    @Override
    public void initialize() {
        // start the shake by openeing to the max shake angle
        intakeArmSubsystem.setAngle(IntakeArmConstants.kShakeMax);
    }   
     
    @Override
    public void execute() {
        // check if the arm has reached the edge angles by checking the state (checking for open and closed in just in case of faliure)

        if (intakeArmSubsystem.getState() == IntakeArmState.SHAKE_MAX || intakeArmSubsystem.getState() == IntakeArmState.OPEN) {
            changeDirection(IntakeArmConstants.kShakeMin);
        }

        if (intakeArmSubsystem.getState() == IntakeArmState.SHAKE_MIN || intakeArmSubsystem.getState() == IntakeArmState.CLOSED) {
            changeDirection(IntakeArmConstants.kShakeMax);
        }
    }

    // changes the rotation direction of the arm
    private void changeDirection(double angle) {
        intakeArmSubsystem.setAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSubsystem.CloseArm();
    }
}