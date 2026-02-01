package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class ShooterSubsystem extends SubsystemBase {

    public enum ShooterState {
        COAST,
        DUTY_CYCLE_BANG_BANG,
        TORQUE_CURRENT_BANG_BANG
    }

    private final SparkFlex m_motor;
    private final SparkFlex s_motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoop;

    private ShooterState state = ShooterState.COAST;
    private WantedState currentWantedState;

    private double targetRPM = Double.NaN;
    private double holdAmps = Constants.ShooterConstants.kHoldAmps;

    // Debouncer to prevent mode flapping
    private final Debouncer torqueDebounce = new Debouncer(Constants.ShooterConstants.kDebouncerTime, DebounceType.kFalling);

    public ShooterSubsystem() {
        m_motor = new SparkFlex(Constants.ShooterConstants.kMainMotorID, MotorType.kBrushless);
        encoder = m_motor.getEncoder();

        closedLoop = m_motor.getClosedLoopController();

        s_motor = new SparkFlex(Constants.ShooterConstants.kSecondaryMotorID, MotorType.kBrushless);

        // Main motor config
        SparkFlexConfig m_config = new SparkFlexConfig();
        m_config.inverted(Constants.ShooterConstants.kInverted)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12)
                .smartCurrentLimit(80);

        m_motor.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        // Secondary motor follows main
        SparkFlexConfig s_config = new SparkFlexConfig();
        s_config.follow(m_motor, !Constants.ShooterConstants.kInverted);
        s_motor.configure(s_config, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        if (Superstructure.getInstance().isSuperstateMode()) handleWantedState();

        if (!Double.isNaN(targetRPM)) {
            handleShootingTarget();
        } else {
            stop();
        }

        // Dashboard
        SmartDashboard.putNumber("Shooter/CurrentRPM", getVelocity());
        SmartDashboard.putNumber("Shooter/TargetRPM", Double.isNaN(targetRPM) ? 0 : targetRPM);
        SmartDashboard.putString("Shooter/State", state.toString());
        SmartDashboard.putNumber("Shooter/CurrentMain", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/CurrentSecondary", s_motor.getOutputCurrent());
    }

    private void handleWantedState() {
        if (currentWantedState == null)
            return;

        switch (currentWantedState) {
            case IDLE:
            case HOME:
            case INTAKING:
            case L1_CLIMB:
            case L3_CLIMB:
                stop();
                break;
            case PREPARING_SHOOTER:
            case SHOOTING:
                handleShooting();
                break;
        }
    }

    private void handleShooting() {
        if (Vision.getInstance().isInAllianceZone()) {
            score();
        } else {
            shootToAllianceZone();
        }
    }

    private void score() {
        setTargetRPM(Superstructure.getInstance().getShooterParameters().flywheelSpeed());
    }

    private void shootToAllianceZone() {
        setTargetRPM(Constants.ShooterConstants.kNeutralZoneShootingRPM);
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }

    public ShooterState getState() {
        return state;
    }

    public boolean isReady() {
        if (currentWantedState == WantedState.PREPARING_SHOOTER || currentWantedState == WantedState.PREPARING_SHOOTER) return state == ShooterState.TORQUE_CURRENT_BANG_BANG;
        return state == ShooterState.COAST;
    }

    public void setTargetRPM(double targetRPM) {
        if (targetRPM != 0) {
            this.targetRPM = targetRPM;
        } else {
            this.targetRPM = Double.NaN;
        }
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public boolean isAtTargetVelocity() {
        if (Double.isNaN(targetRPM))
            return false;
        return Math.abs(getVelocity() - targetRPM) < Constants.ShooterConstants.kTolerance;
    }

    private void handleShootingTarget() {
        boolean inTolerance = isAtTargetVelocity();
        boolean useTorque = torqueDebounce.calculate(inTolerance);

        if (useTorque) {
            // Torque/current mode
            closedLoop.setSetpoint(holdAmps, ControlType.kCurrent);
            state = ShooterState.TORQUE_CURRENT_BANG_BANG;
        } else {
            // Bang-bang open loop
            if (targetRPM < getVelocity()) {
                closedLoop.setSetpoint(1.0, ControlType.kDutyCycle);
            } else {
                closedLoop.setSetpoint(0.0, ControlType.kDutyCycle);
            }
            state = ShooterState.DUTY_CYCLE_BANG_BANG;
        }
    }

    public void stop() {
        targetRPM = Double.NaN;
        m_motor.stopMotor();
        state = ShooterState.COAST;
    }

    public Command setVelocityCommand(double setpoint) {
        return runOnce(() -> setTargetRPM(setpoint));
    }

    @Override
    public void simulationPeriodic() {
    }
}
