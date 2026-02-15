package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.Superstructure.WantedState;

public class IntakeArmSubsystem extends SubsystemBase {
    private final DutyCycleEncoder m_absoluteEncoder;
    private final SparkMax m_motor;
    private SparkClosedLoopController m_controller;
    private final RelativeEncoder m_relativeEncoder;
    private final SysIdRoutine m_SysIdRoutine;
    private final MutVoltage m_appliedVoltage;
    private final MutAngle m_angle;
    private final MutAngularVelocity m_velocity;

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

    private double lastP = IntakeArmConstants.kP;
    private double lastI = IntakeArmConstants.kI;
    private double lastD = IntakeArmConstants.kD;
    private double lastS = IntakeArmConstants.kS;
    private double lastV = IntakeArmConstants.kV;
    private double lastA = IntakeArmConstants.kA;
    private double lastG = IntakeArmConstants.kG;
    private double lastCosRatio = IntakeArmConstants.kCosRatio;

    public IntakeArmSubsystem() {
        // set absolute encoder
        m_absoluteEncoder = new DutyCycleEncoder(
                IntakeArmConstants.kAbsoluteEncoderID,
                IntakeArmConstants.kAbsoluteEncoderRange,
                IntakeArmConstants.kAbsoluteEncoderOffset);

        // set motor
        m_motor = new SparkMax(IntakeArmConstants.kMotorID, MotorType.kBrushless);
        targetAngle = 0;

        // set relative encoder
        m_relativeEncoder = m_motor.getEncoder();

        // set config pid & ff
        SparkMaxConfig config = new SparkMaxConfig();

        // SysId
        m_angle = Radians.mutable(Constants.IntakeArmConstants.kMutTragetAngle);
        m_appliedVoltage = Volts.mutable(Constants.IntakeArmConstants.kMutVolts);
        m_velocity = RadiansPerSecond.mutable(Constants.IntakeArmConstants.kMutVelocity);

        m_SysIdRoutine = new SysIdRoutine(new Config(), new Mechanism(
                m_motor::setVoltage,
                log -> {
                    // Record a frame for the intake motor.
                    log.motor("IntakeArm")
                            .voltage(
                                    m_appliedVoltage.mut_replace(
                                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                            .angularPosition(m_angle.mut_replace(m_relativeEncoder.getPosition(), Rotations))
                            .angularVelocity(
                                    m_velocity.mut_replace(m_relativeEncoder.getVelocity(), RotationsPerSecond));
                }, this

        ));

        SmartDashboard.putNumber("Arm/kP", IntakeArmConstants.kP);
        SmartDashboard.putNumber("Arm/kI", IntakeArmConstants.kI);
        SmartDashboard.putNumber("Arm/kD", IntakeArmConstants.kD);
        SmartDashboard.putNumber("Arm/kS", IntakeArmConstants.kS);
        SmartDashboard.putNumber("Arm/kV", IntakeArmConstants.kV);
        SmartDashboard.putNumber("Arm/kA", IntakeArmConstants.kA);
        SmartDashboard.putNumber("Arm/kG", IntakeArmConstants.kG);
        SmartDashboard.putNumber("Arm/kCosRatio", IntakeArmConstants.kCosRatio);

        // set motor config
        m_motor.configure(config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

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

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
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
        tuning();
        if (currentWantedState != null && Superstructure.getInstance().isSuperstateMode()) {
            SmartDashboard.putNumber("/encoderyippyyy",m_absoluteEncoder.get());
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

    private void tuning() {
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
                    SparkMax.ResetMode.kNoResetSafeParameters,
                    SparkMax.PersistMode.kNoPersistParameters);
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
        }

        if (state == IntakeArmState.SHAKE_MIN || state == IntakeArmState.CLOSED) {
            setAngle(IntakeArmConstants.kShakeMax);
        }
    }

    @Override
    public void simulationPeriodic() {
    }

    public boolean isReady() {
        if (currentWantedState==null) {
            return false;
        }
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
        }
        return false;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}