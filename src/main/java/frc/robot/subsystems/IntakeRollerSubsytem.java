package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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

    private double lastP, lastI, lastD, lastS, lastV, lastA;

    public IntakeRollerSubsytem() {
        m_motor = new SparkMax(Constants.IntakeRollerConstants.kMotorID, MotorType.kBrushless);
        state = IntakeRollerState.IDLE;
        currentWantedState = WantedState.IDLE;

        closedLoop = m_motor.getClosedLoopController();

        SparkMaxConfig m_config = new SparkMaxConfig();

        m_config.smartCurrentLimit(Constants.IntakeRollerConstants.kStallLimit);

        m_config.closedLoop
                .pid(Constants.IntakeRollerConstants.kP, Constants.IntakeRollerConstants.kI,
                        Constants.IntakeRollerConstants.kD).feedForward
                .kS(Constants.IntakeRollerConstants.kS)
                .kV(Constants.IntakeRollerConstants.kV)
                .kA(Constants.IntakeRollerConstants.kA);

        m_config.encoder.positionConversionFactor(Constants.IntakeRollerConstants.kGearRatio)
                .velocityConversionFactor(1 / Constants.IntakeRollerConstants.kGearRatio);

        m_motor.configure(m_config, com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);

        tuningInit();
    }

    private void tuningInit() {
        SmartDashboard.putNumber("IntakeRoller/kP", Constants.IntakeRollerConstants.kP);
        SmartDashboard.putNumber("IntakeRoller/kI", Constants.IntakeRollerConstants.kI);
        SmartDashboard.putNumber("IntakeRoller/kD", Constants.IntakeRollerConstants.kD);
        SmartDashboard.putNumber("IntakeRoller/kS", Constants.IntakeRollerConstants.kS);
        SmartDashboard.putNumber("IntakeRoller/kV", Constants.IntakeRollerConstants.kV);
        SmartDashboard.putNumber("IntakeRoller/kA", Constants.IntakeRollerConstants.kA);

        SmartDashboard.putNumber("IntakeRoller/DebugTargetRPM", 0.0);
    }

    public Command rollerSpinCommand(double rpm) {
        return runOnce(() -> {
            setTargetRpm(rpm);
        });
    }

    public void setTargetRpm(double rpm) {
        state = rpm != 0 ? IntakeRollerState.SPINNING : IntakeRollerState.IDLE;
        System.out.println(rpm);
        if (rpm != 0) {
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
        tuning();

        SmartDashboard.putNumber("IntakeRoller/CurrentRPM", getVelocity());
        SmartDashboard.putNumber("IntakeRoller/TargetRPM", Double.isNaN(targetRPM) ? 0 : targetRPM);

        if (Superstructure.getInstance().isManualMode()) {
            if (!Double.isNaN(targetRPM)) {
                closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
            } else {
                stop();
            }
            return;
        }
        if (currentWantedState == WantedState.INTAKING
                || currentWantedState == WantedState.PREPARING_SHOOTER_AND_INTAKING
                || currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
            if (Superstructure.getInstance().getIntakeState() == IntakeArmState.OPEN)
                setTargetRpm(Constants.IntakeRollerConstants.kIntakePower);
        } else if (currentWantedState == WantedState.SHOOTING) setTargetRpm(Constants.IntakeRollerConstants.kShootingPower);
         else {
            setTargetRpm(0);
        }

        if (!Double.isNaN(targetRPM)) {
            closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
        } else {
            stop();
        }

        
    }

    private void tuning() {
        if (!DriverStation.isTest())
            return;

        double p = SmartDashboard.getNumber("IntakeRoller/kP", Constants.IntakeRollerConstants.kP);
        double i = SmartDashboard.getNumber("IntakeRoller/kI", Constants.IntakeRollerConstants.kI);
        double d = SmartDashboard.getNumber("IntakeRoller/kD", Constants.IntakeRollerConstants.kD);
        double s = SmartDashboard.getNumber("IntakeRoller/kS", Constants.IntakeRollerConstants.kS);
        double v = SmartDashboard.getNumber("IntakeRoller/kV", Constants.IntakeRollerConstants.kV);
        double a = SmartDashboard.getNumber("IntakeRoller/kA", Constants.IntakeRollerConstants.kA);

        // Check for changes
        if (p != lastP || i != lastI || d != lastD || s != lastS || v != lastV || a != lastA) {
            lastP = p;
            lastI = i;
            lastD = d;
            lastS = s;
            lastV = v;
            lastA = a;

            SparkMaxConfig tuneConfig = new SparkMaxConfig();
            tuneConfig.closedLoop
                    .pid(p, i, d).feedForward.kS(s).kV(v).kA(a);

            // Apply live without reset
            m_motor.configure(tuneConfig,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);

            System.out.println("Roller: Tuned in new values! 🌪️");
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean isReady() {
        if (currentWantedState != WantedState.INTAKING
                && currentWantedState != WantedState.PREPARING_SHOOTER_AND_INTAKING
                && currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
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
