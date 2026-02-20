package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;

public class IntakeRollerSubsytem extends SubsystemBase {
    public enum IntakeRollerState {
        SPINNING,
        IDLE
    }

    private final SparkMax m_motor;
    private IntakeRollerState state;
    private WantedState currentWantedState;

    private final SparkClosedLoopController closedLoop;

    private double targetRPM = Double.NaN;


    public IntakeRollerSubsytem() {
        m_motor = new SparkMax(Constants.IntakeRollerConstants.kMotorID, MotorType.kBrushless);
        state = IntakeRollerState.IDLE;
        currentWantedState=WantedState.IDLE;

        closedLoop = m_motor.getClosedLoopController();

        SparkMaxConfig m_config = new SparkMaxConfig();

        m_config.closedLoop
            .pid(Constants.IntakeRollerConstants.kP, Constants.IntakeRollerConstants.kI, Constants.IntakeRollerConstants.kD)
                .feedForward
                    .kS(Constants.IntakeRollerConstants.kS)
                    .kV(Constants.IntakeRollerConstants.kV)
                    .kA(Constants.IntakeRollerConstants.kA);

        m_config.encoder.positionConversionFactor(Constants.IntakeRollerConstants.kGearRatio)
                .velocityConversionFactor(1 / Constants.IntakeRollerConstants.kGearRatio);

        m_motor.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    public Command rollerSpinCommand(double rpm) {
        return runOnce(() -> {
            setTargetRpm(rpm);
        });
    }

    public void setTargetRpm(double rpm) {
        state = rpm != 0 ? IntakeRollerState.SPINNING : IntakeRollerState.IDLE;
        System.out.println(rpm);
        if (targetRPM != 0) {
            targetRPM = rpm;
        } else {
            targetRPM = Double.NaN;
        }
    }

    public IntakeRollerState getState() {
        return this.state;
    }

    @Override
    public void periodic() {
        if (Superstructure.getInstance().isManualMode()) return;
        if (currentWantedState == WantedState.INTAKING || currentWantedState == WantedState.PREPARING_SHOOTER_AND_INTAKING || currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
            if (Superstructure.getInstance().getIntakeState() == IntakeArmState.OPEN) setTargetRpm(Constants.IntakeRollerConstants.kIntakePower);
        }
        else {
            setTargetRpm(0);
        }

        if (!Double.isNaN(targetRPM)) {
        closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
    } else {
        stop();
    }

        SmartDashboard.putNumber("IntakeRoller/CurrentRPM", getVelocity());
        SmartDashboard.putNumber("IntakeRoller/TargetRPM", Double.isNaN(targetRPM) ? 0 : targetRPM);
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean isReady() {
        if (currentWantedState != WantedState.INTAKING && currentWantedState != WantedState.PREPARING_SHOOTER_AND_INTAKING && currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
            return state == IntakeRollerState.IDLE;
        }
        return state == IntakeRollerState.SPINNING;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }

    public double getVelocity() {
        return m_motor.getEncoder().getVelocity();
    }

public void stop() {
        targetRPM = Double.NaN;
        m_motor.stopMotor();
        state = IntakeRollerState.IDLE;
    }
}
