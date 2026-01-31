package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    public enum IntakeArmState {
        IDLE,
        OPEN,
        CLOSED,
        MOVING,
        SHAKE_MAX,
        SHAKE_MIN
    }

    public IntakeArmState state = IntakeArmState.IDLE;
    private WantedState currentWantedState;

    public IntakeArmSubsystem() {
        // set absolute encoder
        m_absoluteEncoder = new DutyCycleEncoder(
                IntakeArmConstants.kAbsoluteEncoderID,
                IntakeArmConstants.kAbsoluteEncoderRange,
                IntakeArmConstants.kAbsoluteEncoderOffset);

        // set motor
        m_motor = new SparkMax(IntakeArmConstants.kMotorID, MotorType.kBrushless);
        targetAngle = 0;

        // set config pid & ff
        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                        IntakeArmConstants.kP,
                        IntakeArmConstants.kI,
                        IntakeArmConstants.kD).feedForward
                .kS(IntakeArmConstants.kS)
                .kV(IntakeArmConstants.kV)
                .kA(IntakeArmConstants.kA)
                .kCos(IntakeArmConstants.kG)
                .kCosRatio(IntakeArmConstants.kCosRatio);

        // set motor config
        m_motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

        // set relative encoder
        m_relativeEncoder = m_motor.getEncoder();

        // set motor contoller
        m_controller = m_motor.getClosedLoopController();

        // set the relative position to the duty position
        m_relativeEncoder.setPosition(m_absoluteEncoder.get());
    }

    public Command IntakeArmCommand(double angle) {
        return runOnce(() -> {
            setAngle(angle);
        });
    }

    // set target angle
    public void setAngle(double angle) {
        this.targetAngle = angle;
        state = IntakeArmState.MOVING;
    }

    public IntakeArmState getState() {
        return state;
    }

    public double getAngle() {
        return m_relativeEncoder.getPosition();
    }

    private boolean isOpen() {
        return Math.abs(IntakeArmConstants.kOpenAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }

    // returns true if the arm is in the max shaking angle
    private boolean isAtShakeMax() {
        return Math.abs(IntakeArmConstants.kShakeMax - getAngle()) < IntakeArmConstants.kTolerance;
    }

    // returns true if the arm is in the minimum shaking angle
    private boolean isAtShakeMin() {
        return Math.abs(IntakeArmConstants.kShakeMin - getAngle()) < IntakeArmConstants.kTolerance;
    } 

    public void OpenArm() {
        setAngle(IntakeArmConstants.kOpenAngle);
    }

    private boolean isClosed() {
        return Math.abs(IntakeArmConstants.kClosedAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }

    public void CloseArm() {
        setAngle(IntakeArmConstants.kClosedAngle);
    }

    @Override
    public void periodic() {
        if (currentWantedState!=null){
          handleWantedState();
        }

        m_controller.setSetpoint(this.targetAngle, ControlType.kPosition);

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
        }
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
            case INTAKING:
                OpenArm();
                break;
            case SHOOTING:
                handleArmShake();    
                break;
        }
    }    

    private void handleArmShake(){
        
        if (state == IntakeArmState.SHAKE_MAX || state == IntakeArmState.OPEN) {
            setAngle(IntakeArmConstants.kShakeMin);
        }

        if (state == IntakeArmState.SHAKE_MIN || state == IntakeArmState.CLOSED) {
            setAngle(IntakeArmConstants.kShakeMax);
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean isReady() {
         switch (currentWantedState) {
            case IDLE:
            case HOME:
            case PREPARING_SHOOTER:
            case L1_CLIMB:
            case L3_CLIMB:
                return state==IntakeArmState.CLOSED;
            case INTAKING:
                return state==IntakeArmState.OPEN;
            case SHOOTING:
                return state==IntakeArmState.OPEN || state==IntakeArmState.CLOSED;

        }
        return false;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}