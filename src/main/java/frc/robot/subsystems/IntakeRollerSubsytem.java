package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;

public class IntakeRollerSubsytem extends SubsystemBase {
    public enum IntakeRollerState {
        SPINNING,
        IDLE
    }

    private final SparkMax m_motor;
    private IntakeRollerState state;
    private WantedState currentWantedState;


    public IntakeRollerSubsytem() {
        m_motor = new SparkMax(Constants.IntakeRollerConstants.kMotorID, MotorType.kBrushless);
        state = IntakeRollerState.IDLE;
    }

    public Command rollerSpinCommand(double voltage) {
        return runOnce(() -> {
            setVoltage(voltage);
        });
    }

    public void setVoltage(double voltage) {
        state = voltage != 0 ? IntakeRollerState.SPINNING : IntakeRollerState.IDLE;
        m_motor.setVoltage(voltage);
    }

    public IntakeRollerState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        if (Superstructure.getInstance().isManualMode()) return;
        if (currentWantedState == WantedState.INTAKING && Superstructure.getInstance().getIntakeState() == IntakeArmState.OPEN) {
            setVoltage(Constants.IntakeRollerConstants.kIntakePower);
        }
        else {
            setVoltage(0);
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean isReady() {
        if (currentWantedState != WantedState.INTAKING) {
            return state == IntakeRollerState.IDLE;
        }
        return state == IntakeRollerState.SPINNING;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}
