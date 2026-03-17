package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

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
        VELOCITY_CONTROL
    }

    private final SparkFlex m_motor;
    private final SparkFlex s_motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoop;

    private ShooterState state = ShooterState.COAST;
    private WantedState currentWantedState;
    private double targetRPM = Double.NaN;
    private double targetSurfaceSpeed = Double.NaN;

    private SwerveSubsystem swerveSubsystem;

    public ShooterSubsystem() {
        m_motor = new SparkFlex(ShooterConstants.kMainMotorID, MotorType.kBrushless);
        encoder = m_motor.getEncoder();
        closedLoop = m_motor.getClosedLoopController();
        s_motor = new SparkFlex(ShooterConstants.kSecondaryMotorID, MotorType.kBrushless);

        SparkFlexConfig m_config = new SparkFlexConfig();
        m_config.inverted(ShooterConstants.kInverted)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(12);

        m_config.closedLoop
                .pid(ShooterConstants.kP,
                        ShooterConstants.kI,
                        ShooterConstants.kD).feedForward
                .kS(ShooterConstants.kS)
                .kV(ShooterConstants.kV)
                .kA(ShooterConstants.kA);

        m_config.smartCurrentLimit(ShooterConstants.kStallLimit);

        m_motor.configure(m_config,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        SparkFlexConfig s_config = new SparkFlexConfig();
        s_config.follow(m_motor, true);
        s_motor.configure(s_config,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter/Debug Speed", 0);
    }

    public void setSwerveSubsystem(SwerveSubsystem swerve) {
        this.swerveSubsystem = swerve;
    }

    @Override
    public void periodic() {
        publishTelemetry();

        if (DriverStation.isTest()) {
            runTuningMode();
            return;
        }

        if (Superstructure.getInstance().isSuperstateMode()) {
            handleWantedState();
        }

        if (!Double.isNaN(targetRPM) && targetRPM > 0) {
            handleShootingTarget();
        } else {
            m_motor.stopMotor();
            state = ShooterState.COAST;
        }
    }

    private void handleWantedState() {
        if (currentWantedState == null)
            return;

        switch (currentWantedState) {
            case IDLE:
            case HOME:
            case INTAKING:
            case L1_CLOSE_AUTON:
            case L1_CLOSE_TELEOP:
            case L1_EXTEND_AUTON:
            case L1_EXTEND_TELEOP:
                stop();
                break;
            case PREPARING_SHOOTER_AND_INTAKING:
            case SHOOTING_AND_INTAKING:
            case PREPARING_SHOOTER:
            case SHOOTING:
                handleShooting();
                break;
            case UNJAM:
                setTargetRPM(Constants.ShooterConstants.kUnjamPower);
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

    /**
     * Computes shot RPM using Vision's compensated flywheel speed.
     *
     * Replaces the flat Superstructure.getShooterParameters().flywheelSpeed() call.
     * Now uses kShotFlywheelSpeedMap (distance lookup) + kRadialRPMComp (velocity
     * compensation that was always in Constants but never wired in until now).
     *
     * Falls back to Superstructure parameter if swerveSubsystem not yet set.
     */
    private void score() {
        if (swerveSubsystem != null) {
            setTargetSurfaceSpeed(Vision.getInstance()
                    .getCompensatedFlywheelRPM());
        } else {
            setTargetSurfaceSpeed(ShooterConstants.kMaxSpeed);
        }
    }

    private void shootToAllianceZone() {
        setTargetRPM(Constants.ShooterConstants.kNeutralZoneShootingSpeed);
    }

    private void handleShootingTarget() {
        closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
        state = ShooterState.VELOCITY_CONTROL;
    }

    private void runTuningMode() {
        double debugSpeed = SmartDashboard.getNumber("Shooter/Debug Speed", 0);
        setTargetSurfaceSpeed(debugSpeed);
        if (debugSpeed == 0) {
            m_motor.stopMotor();
            state = ShooterState.COAST;
        } else {
            closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
            state = ShooterState.VELOCITY_CONTROL;
        }
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("Shooter/CurrentRPM", getVelocity());
        SmartDashboard.putNumber("Shooter/TargetRPM", Double.isNaN(targetRPM) ? 0 : targetRPM);
        SmartDashboard.putNumber("Shooter/Target Surface Speed", Double.isNaN(targetSurfaceSpeed) ? 0 : targetSurfaceSpeed);
        SmartDashboard.putNumber("Shooter/Current Surface Speed", getSurfaceVelocity());
        SmartDashboard.putString("Shooter/State", state.toString());
        SmartDashboard.putNumber("Shooter/CurrentMain_A", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/CurrentSecond_A", s_motor.getOutputCurrent());
        SmartDashboard.putBoolean("Shooter/AtTarget", isAtTargetVelocity());
        SmartDashboard.putBoolean("Shooter/Ready", isReady());
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }

    public ShooterState getState() {
        return state;
    }

    /**
     * Shooter is at target velocity and in a shooting state.
     *
     * Full "ready to fire" gate (use both together):
     * boolean fire = shooterSubsystem.isReady() && swerveSubsystem.isAimAligned();
     */
    public boolean isReady() {
        if (currentWantedState == WantedState.PREPARING_SHOOTER
                || currentWantedState == WantedState.PREPARING_SHOOTER_AND_INTAKING
                || currentWantedState == WantedState.SHOOTING_AND_INTAKING
                || currentWantedState == WantedState.SHOOTING) {
            return state == ShooterState.VELOCITY_CONTROL && isAtTargetVelocity();
        }
        return state == ShooterState.COAST;
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = (targetRPM != 0) ? targetRPM : Double.NaN;
        this.targetSurfaceSpeed =  (targetRPM * Math.PI * ShooterConstants.kWheelDiameterMeters) / 60;
    }

    public void setTargetSurfaceSpeed(double surfaceSpeed) {
        this.targetSurfaceSpeed = surfaceSpeed != 0 ? surfaceSpeed : Double.NaN;
        this.targetRPM = surfaceSpeed != 0 ? (surfaceSpeed * 60) / (Math.PI * ShooterConstants.kWheelDiameterMeters) : Double.NaN;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getSurfaceVelocity() {
        return (getVelocity() * Math.PI * ShooterConstants.kWheelDiameterMeters) / 60;
    }

    public boolean isAtTargetVelocity() {
        if (Double.isNaN(targetRPM))
            return false;
        return Math.abs(getVelocity() - targetRPM) < Constants.ShooterConstants.kTolerance;
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