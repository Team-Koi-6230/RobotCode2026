package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double lastP, lastI, lastD, lastS, lastV, lastA;

    public FeederSubsystem() {
        m_SparkMax = new SparkMax(Constants.FeederConstants.kMotorID, MotorType.kBrushless);
        state = FeederState.IDLE;
        currentWantedState = WantedState.IDLE;

        closedLoop = m_SparkMax.getClosedLoopController();

        SparkMaxConfig m_config = new SparkMaxConfig();

        m_config.smartCurrentLimit(Constants.FeederConstants.kStallLimit);

        m_config.encoder.positionConversionFactor(1 / Constants.FeederConstants.kGearRatio)
                .velocityConversionFactor((1 / Constants.FeederConstants.kGearRatio));

        m_config.closedLoop
                .pid(Constants.FeederConstants.kP, Constants.FeederConstants.kI,
                        Constants.FeederConstants.kD).feedForward
                .kS(Constants.FeederConstants.kS)
                .kV(Constants.FeederConstants.kV)
                .kA(Constants.FeederConstants.kA);

        m_SparkMax.configure(m_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        tuningInit();
    }

    private void tuningInit() {
        SmartDashboard.putNumber("Feeder/kP", Constants.FeederConstants.kP);
        SmartDashboard.putNumber("Feeder/kI", Constants.FeederConstants.kI);
        SmartDashboard.putNumber("Feeder/kD", Constants.FeederConstants.kD);
        SmartDashboard.putNumber("Feeder/kS", Constants.FeederConstants.kS);
        SmartDashboard.putNumber("Feeder/kV", Constants.FeederConstants.kV);
        SmartDashboard.putNumber("Feeder/kA", Constants.FeederConstants.kA);

        SmartDashboard.putNumber("Feeder/DebugTargetRPM", 0.0);
    }

    public Command feederSpinCommand(double rpm) {
        return runOnce(() -> {
            setTargetRpm(rpm);
        });
    }

    public void setTargetRpm(double rpm) {
        state = rpm != 0 ? FeederState.SPINNING : FeederState.IDLE;

        if (rpm != 0) {
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
        tuning();

        if (Superstructure.getInstance().isSuperstateMode()) {
            if (currentWantedState == WantedState.SHOOTING || currentWantedState == WantedState.SHOOTING_AND_INTAKING) {
                setTargetRpm(Constants.FeederConstants.kGrabRpm);
            } else if(currentWantedState == WantedState.UNJAM) {
                setTargetRpm(-Constants.FeederConstants.kGrabRpm);
            } else {
                setTargetRpm(0);
            }
        }

        if (!Double.isNaN(targetRPM))
            closedLoop.setSetpoint(targetRPM, ControlType.kVelocity);
        else
            stop();
    }

    private void tuning() {
        if (!DriverStation.isTest())
            return;

        SmartDashboard.putNumber("Feeder/targetRPM", targetRPM);
        SmartDashboard.putNumber("Feeder/CurrentRPM", m_SparkMax.getEncoder().getVelocity());

        double p = SmartDashboard.getNumber("Feeder/kP", Constants.FeederConstants.kP);
        double i = SmartDashboard.getNumber("Feeder/kI", Constants.FeederConstants.kI);
        double d = SmartDashboard.getNumber("Feeder/kD", Constants.FeederConstants.kD);
        double s = SmartDashboard.getNumber("Feeder/kS", Constants.FeederConstants.kS);
        double v = SmartDashboard.getNumber("Feeder/kV", Constants.FeederConstants.kV);
        double a = SmartDashboard.getNumber("Feeder/kA", Constants.FeederConstants.kA);

        if (p != lastP || i != lastI || d != lastD || s != lastS || v != lastV || a != lastA) {
            lastP = p;
            lastI = i;
            lastD = d;
            lastS = s;
            lastV = v;
            lastA = a;

            SparkMaxConfig tuneConfig = new SparkMaxConfig();
            tuneConfig.closedLoop
                    .pid(p, i, d).feedForward
                    .kS(s)
                    .kV(v)
                    .kA(a);

            m_SparkMax.configure(tuneConfig,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);

            System.out.println("Feeder: PID + FF Updated! 🌪️");
        }

        double debugRPM = SmartDashboard.getNumber("Feeder/DebugTargetRPM", 0.0);
        if (debugRPM != 0)
            setTargetRpm(debugRPM);
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
        if (currentWantedState != WantedState.SHOOTING || currentWantedState == WantedState.SHOOTING_AND_INTAKING || currentWantedState != WantedState.UNJAM) {
            return state == FeederState.IDLE;
        }
        return state == FeederState.SPINNING;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}
