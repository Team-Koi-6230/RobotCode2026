package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class RumbleSubsystem extends SubsystemBase {
    public enum Priority {
        NONE, LOW, MEDIUM, HIGH
    }

    private CommandXboxController m_driverController;
    private CommandXboxController m_operatorController;

    private double currentStrength;
    private double endTime;
    private Priority currentPriority;


    public RumbleSubsystem() {
        this.currentStrength = 0;
        this.endTime = 0;
        this.currentPriority = Priority.NONE;
    }

    public void setControllers(CommandXboxController m_driverController, CommandXboxController m_operatorController) {
        this.m_driverController = m_driverController;
        this.m_operatorController = m_operatorController;
    }

    public void rumble(double strength, double seconds, Priority priority) {
        if (currentPriority.ordinal() > priority.ordinal()) {
            System.out.println("Skipping rumble due to low priority");
            return;
        }
        setRumble(0);
        currentStrength = strength;
        endTime = Timer.getFPGATimestamp() + seconds;
        currentPriority = priority;
    }

    public void rumble(RumblePack rumblePack) {
        rumble(rumblePack.strength, rumblePack.duration, rumblePack.priority);
    } 

    public void changeStateRumble() {
        rumble(Constants.OperatorConstants.kStateChangeRumble);
    }

    private void setRumble(double strength) {
        if (m_driverController == null || m_operatorController == null) return;
        m_driverController.setRumble(GenericHID.RumbleType.kBothRumble, strength);
        m_operatorController.setRumble(GenericHID.RumbleType.kBothRumble, strength);
    }


    @Override
    public void periodic() {
        if (Timer.getFPGATimestamp() < endTime) {
            setRumble(currentStrength);
        } else {
            setRumble(0);
            currentPriority = Priority.NONE;
        }
    }
}
