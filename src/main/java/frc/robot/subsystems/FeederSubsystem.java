package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;

public class FeederSubsystem extends SubsystemBase {
    public enum FeederState {
        SPINNING,
        IDLE
    }

    private final SparkMax m_SparkMax;
    private FeederState state;
    private WantedState currentWantedState;
    private final SparkClosedLoopController closedLoop;

    private double targetRPM = Double.NaN;

    public FeederSubsystem() {
        m_SparkMax = new SparkMax(Constants.FeederConstants.kMotorID, MotorType.kBrushless);
        state = FeederState.IDLE;
        currentWantedState = WantedState.IDLE;

        closedLoop = m_SparkMax.getClosedLoopController();

        SparkMaxConfig m_config = new SparkMaxConfig();

        m_config.closedLoop
                .pid(Constants.FeederConstants.kP, Constants.FeederConstants.kI,
                        Constants.FeederConstants.kD).feedForward
                .kS(Constants.FeederConstants.kS)
                .kV(Constants.FeederConstants.kV)
                .kA(Constants.FeederConstants.kA);
    }

    public Command feederSpinCommand(double rpm) {
        return runOnce(() -> {
            setTargetRpm(rpm);
        });
    }

    public void setTargetRpm(double rpm) {
        state = rpm != 0 ? FeederState.SPINNING : FeederState.IDLE;

        if (targetRPM != 0) {
            targetRPM = rpm;
        } else {
            targetRPM = Double.NaN;
        }
    }

    public FeederState getState() {
        return state;
    }

    @Override
    public void periodic() {
        if (Superstructure.getInstance().isSuperstateMode()) {
            if (currentWantedState == WantedState.SHOOTING || currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
                setTargetRpm(Constants.FeederConstants.kGrabRpm);
            } else {
                setTargetRpm(0);
            }
        }

        if (!Double.isNaN(targetRPM))
            closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
        else
            stop();
    }

    public void stop() {
        targetRPM = Double.NaN;
        m_SparkMax.stopMotor();
        state = FeederState.IDLE;
    }

    @Override
    public void simulationPeriodic() {
        // Called once per scheduler run during simulation
    }

    public boolean isReady() {
        if (currentWantedState != WantedState.SHOOTING || currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
            return state == FeederState.IDLE;
        }
        return state == FeederState.SPINNING;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}
