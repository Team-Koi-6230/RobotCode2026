package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.Superstructure.WantedState;

public class IntakeArmSubsystem extends SubsystemBase {
    private final DutyCycleEncoder m_absoluteEncoder;
    private final SparkMax m_motor;
    private SparkClosedLoopController m_controller;
    private final RelativeEncoder m_relativeEncoder;

    private double targetAngle;
    private int syncCounter = 0; // To avoid syncing every single frame

    public enum IntakeArmState {
        IDLE, OPEN, CLOSED, MOVING, SHAKE_MAX, SHAKE_MIN
    }

    public IntakeArmState state = IntakeArmState.IDLE;
    private WantedState currentWantedState;

    // Tuning variables
    private double lastP, lastI, lastD, lastS, lastV, lastA, lastG, lastCosRatio;

    public IntakeArmSubsystem() {
        m_absoluteEncoder = new DutyCycleEncoder(
                IntakeArmConstants.kAbsoluteEncoderID,
                IntakeArmConstants.kAbsoluteEncoderRange,
                IntakeArmConstants.kAbsoluteEncoderOffset);

        m_motor = new SparkMax(IntakeArmConstants.kMotorID, MotorType.kBrushless);
        targetAngle = Double.NaN;
        m_relativeEncoder = m_motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
                .pid(IntakeArmConstants.kP, IntakeArmConstants.kI, IntakeArmConstants.kD).feedForward
                .kS(IntakeArmConstants.kS)
                .kV(IntakeArmConstants.kV)
                .kA(IntakeArmConstants.kA)
                .kCos(IntakeArmConstants.kG)
                .kCosRatio(IntakeArmConstants.kCosRatio);
        config.closedLoop.maxMotion.
        allowedProfileError(0.5)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .maxAcceleration(0.5)
        .cruiseVelocity(1);

        config.encoder.positionConversionFactor(IntakeArmConstants.kGearRatio);

        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_controller = m_motor.getClosedLoopController();

        // Initial sync so we aren't starting at zero
        m_relativeEncoder.setPosition(m_absoluteEncoder.get());
    }

    @Override
    public void periodic() {
        tuning();

        if (currentWantedState != null && Superstructure.getInstance().isSuperstateMode()) {
            handleWantedState();
        } 

        if (!Double.isNaN(targetAngle)) {
            m_controller.setSetpoint(this.targetAngle, ControlType.kMAXMotionPositionControl);
        } else {
            m_motor.stopMotor();
        }

        syncEncodersWithThreshold();
        updateState();
    }

    /**
     * Re-syncs the relative encoder to the absolute encoder if they drift,
     * but only if the arm is stationary to avoid PID "jumps".
     */
    private void syncEncodersWithThreshold() {
        double currentRel = getAngle();
        double currentAbs = m_absoluteEncoder.get();
        double velocity = m_relativeEncoder.getVelocity();

        // Only sync if stationary and error is > threshold (e.g. 0.02 units)
        if (Math.abs(velocity) < 0.01 && Math.abs(currentRel - currentAbs) > 0.02 && Math.abs(targetAngle - getAngle()) < IntakeArmConstants.kTolerance) {
            syncCounter++;
            if (syncCounter > 10) {
                m_relativeEncoder.setPosition(currentAbs);
                syncCounter = 0;
            }
        } else {
            syncCounter = 0;
        }
    }

    private void updateState() {
        if (Double.isNaN(this.targetAngle)) {
            state = IntakeArmState.IDLE;
        } else if (isOpen()) {
            state = IntakeArmState.OPEN;
        } else if (isClosed()) {
            state = IntakeArmState.CLOSED;
        } else if (isAtShakeMax()) {
            state = IntakeArmState.SHAKE_MAX;
        } else if (isAtShakeMin()) {
            state = IntakeArmState.SHAKE_MIN;
        } else {
            state = IntakeArmState.MOVING;
        }
    }

    public void setAngle(double angle) {
        this.targetAngle = angle;
    }

    public double getAngle() {
        return m_relativeEncoder.getPosition();
    }

    private boolean isOpen() {
        return Math.abs(IntakeArmConstants.kOpenAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }

    private boolean isClosed() {
        return Math.abs(IntakeArmConstants.kClosedAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }

    private boolean isAtShakeMax() {
        return Math.abs(IntakeArmConstants.kShakeMax - getAngle()) < IntakeArmConstants.kTolerance;
    }

    private boolean isAtShakeMin() {
        return Math.abs(IntakeArmConstants.kShakeMin - getAngle()) < IntakeArmConstants.kTolerance;
    }

    public void OpenArm() {
        setAngle(IntakeArmConstants.kOpenAngle);
    }

    public void CloseArm() {
        setAngle(IntakeArmConstants.kClosedAngle);
    }

    private void handleWantedState() {
        switch (currentWantedState) {
            case IDLE:
            case HOME:
            case PREPARING_SHOOTER:
            case L1_CLIMB:
            case L3_CLIMB:
                CloseArm();
                break;
            case PREPARING_SHOOTER_AND_INTAKING:
            case SHOOTING_AND_INTAKING:
            case INTAKING:
                OpenArm();
                break;
            case SHOOTING:
                handleArmShake();
                break;
        }
    }

    private void handleArmShake() {
        if (state == IntakeArmState.SHAKE_MAX || state == IntakeArmState.OPEN) {
            setAngle(IntakeArmConstants.kShakeMin);
        } else if (state == IntakeArmState.SHAKE_MIN || state == IntakeArmState.CLOSED) {
            setAngle(IntakeArmConstants.kShakeMax);
        }
    }

    private void tuning() {
        SmartDashboard.putBoolean("Arm/Is dutycycle encoder connected", m_absoluteEncoder.isConnected());
        SmartDashboard.putNumber("Arm/Abs encoder angle", m_absoluteEncoder.get());
        SmartDashboard.putNumber("Arm/Rel encoder angle", getAngle());
        SmartDashboard.putNumber("Arm/TargetAngle", targetAngle);
        SmartDashboard.putBoolean("Arm/is at position", Math.abs(targetAngle - getAngle()) < IntakeArmConstants.kTolerance);
        SmartDashboard.putNumber("Arm/Error", targetAngle - getAngle());
        double p = SmartDashboard.getNumber("Arm/kP", IntakeArmConstants.kP);
        double i = SmartDashboard.getNumber("Arm/kI", IntakeArmConstants.kI);
        double d = SmartDashboard.getNumber("Arm/kD", IntakeArmConstants.kD);
        double s = SmartDashboard.getNumber("Arm/kS", IntakeArmConstants.kS);
        double v = SmartDashboard.getNumber("Arm/kV", IntakeArmConstants.kV);
        double a = SmartDashboard.getNumber("Arm/kA", IntakeArmConstants.kA);
        double g = SmartDashboard.getNumber("Arm/kG", IntakeArmConstants.kG);
        double cosRatio = SmartDashboard.getNumber("Arm/kCosRatio", IntakeArmConstants.kCosRatio);

        if (p != lastP || i != lastI || d != lastD || s != lastS ||
                v != lastV || a != lastA || g != lastG || cosRatio != lastCosRatio) {
            lastP = p;
            lastI = i;
            lastD = d;  
            lastS = s;
            lastV = v;
            lastA = a;
            lastG = g;
            lastCosRatio = cosRatio;
            SparkMaxConfig tuneConfig = new SparkMaxConfig();
            tuneConfig.closedLoop
                    .pid(p, i, d).feedForward
                    .kS(s)
                    .kV(v)
                    .kA(a)
                    .kCos(g)
                    .kCosRatio(cosRatio);

            // Apply changes without a hard reset
            m_motor.configure(tuneConfig,
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);

        }

    }

    public Command IntakeArmCommand(double angle) {
        return runOnce(() -> setAngle(angle));
    }

    public boolean isReady() {
        if (currentWantedState == null)
            return false;
        switch (currentWantedState) {
            case IDLE:
            case HOME:
            case PREPARING_SHOOTER:
            case L1_CLIMB:
            case L3_CLIMB:
                return state == IntakeArmState.CLOSED;
            case INTAKING:
            case SHOOTING_AND_INTAKING:
            case PREPARING_SHOOTER_AND_INTAKING:
                return state == IntakeArmState.OPEN;
            case SHOOTING:
                return state == IntakeArmState.OPEN || state == IntakeArmState.CLOSED;
            default:
                return false;
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }

    public IntakeArmState getState() {
        return state;
    }
}