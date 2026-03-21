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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
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
    private double _lastTargetAngle;
    private int currentStep;
    private double targetDistance;
    private int syncCounter = 0;
    private boolean isMovingOut = true; // Added for the shake state machine

    public enum IntakeArmState {
        IDLE, OPEN, CLOSED, MOVING, SHAKE
    }

    public IntakeArmState state = IntakeArmState.IDLE;
    private WantedState currentWantedState;

    // Tuning variables
    private double lastP, lastI, lastD, lastS, lastV, lastA, lastG, lastCosRatio, lastMaxVel, lastMaxAccel;

    public IntakeArmSubsystem() {
        currentStep = 1;

        m_absoluteEncoder = new DutyCycleEncoder(
                IntakeArmConstants.kAbsoluteEncoderID,
                IntakeArmConstants.kAbsoluteEncoderRange,
                IntakeArmConstants.kAbsoluteEncoderOffset);

        m_motor = new SparkMax(IntakeArmConstants.kMotorID, MotorType.kBrushless);
        targetAngle = Double.NaN;
        m_relativeEncoder = m_motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(20).softLimit
                .forwardSoftLimit(IntakeArmConstants.kForwardSoftLimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(IntakeArmConstants.kReverseSoftLimit)
                .reverseSoftLimitEnabled(true);

        config.closedLoop
                .pid(IntakeArmConstants.kP, IntakeArmConstants.kI, IntakeArmConstants.kD).feedForward
                .kS(IntakeArmConstants.kS)
                .kV(IntakeArmConstants.kV)
                .kA(IntakeArmConstants.kA)
                .kCos(IntakeArmConstants.kG)
                .kCosRatio(IntakeArmConstants.kCosRatio);

        config.closedLoop.maxMotion
                .maxAcceleration(IntakeArmConstants.kMaxAcceleration)
                .allowedProfileError(IntakeArmConstants.kTolerance)
                .cruiseVelocity(IntakeArmConstants.kCruiseVelocity)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        config.encoder.positionConversionFactor(360.0 / IntakeArmConstants.kGearRatio)
                .velocityConversionFactor((360.0 / IntakeArmConstants.kGearRatio) / 60.0);

        m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_controller = m_motor.getClosedLoopController();

        // Initial sync so we aren't starting at zero
        m_relativeEncoder.setPosition(m_absoluteEncoder.get());

        tuningInit();
    }

    private void tuningInit() {
        SmartDashboard.putNumber("Arm/kP", IntakeArmConstants.kP);
        SmartDashboard.putNumber("Arm/kI", IntakeArmConstants.kI);
        SmartDashboard.putNumber("Arm/kD", IntakeArmConstants.kD);
        SmartDashboard.putNumber("Arm/kS", IntakeArmConstants.kS);
        SmartDashboard.putNumber("Arm/kV", IntakeArmConstants.kV);
        SmartDashboard.putNumber("Arm/kA", IntakeArmConstants.kA);
        SmartDashboard.putNumber("Arm/kG", IntakeArmConstants.kG);
        SmartDashboard.putNumber("Arm/kCosRatio", IntakeArmConstants.kCosRatio);

        SmartDashboard.putNumber("Arm/Max Velocity", IntakeArmConstants.kCruiseVelocity);
        SmartDashboard.putNumber("Arm/Max Accel", IntakeArmConstants.kMaxAcceleration);
    }

    @Override
    public void periodic() {
        tuning();

        if (currentWantedState != null && Superstructure.getInstance().isSuperstateMode()) {
            handleWantedState();
        }

        if (!Double.isNaN(targetAngle)) {
            double error = Math.abs(targetAngle - getAngle());
            if (error < 2) {
                m_controller.setSetpoint(0, ControlType.kVoltage);
                SmartDashboard.putBoolean("Arm/is stopping", true);
            } else {
                m_controller.setSetpoint(targetAngle, ControlType.kMAXMotionPositionControl);
                SmartDashboard.putBoolean("Arm/is stopping", false);
            }
        } else {
            m_motor.stopMotor();
        }

        syncEncodersWithThreshold();
        updateState();
        if (_lastTargetAngle != targetAngle) {
            m_relativeEncoder.setPosition(m_absoluteEncoder.get());
            _lastTargetAngle = targetAngle;
        }

        // reset the current step counter and direction when not shaking
        if (state != IntakeArmState.SHAKE) {
            currentStep = 1;
            isMovingOut = isOpen() ? false : true;
        }

        SmartDashboard.putNumber("Arm/Rel encoder angle", getAngle());
        SmartDashboard.putNumber("Arm/TargetAngle", targetAngle);
    }

    private void updateState() {
        if (Double.isNaN(this.targetAngle)) {
            state = IntakeArmState.IDLE;
        } else if (isOpen()) {
            state = IntakeArmState.OPEN;
        } else if (isClosed()) {
            state = IntakeArmState.CLOSED;
        } else if (currentWantedState == WantedState.SHOOTING) {
            state = IntakeArmState.SHAKE;
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

        private void syncEncodersWithThreshold() {
        double currentRel = getAngle();
        double currentAbs = m_absoluteEncoder.get();
        double velocity = m_relativeEncoder.getVelocity();

        // Only sync if stationary and error is > threshold (e.g. 0.02 units)
        if (Math.abs(velocity) < 0.01 && Math.abs(currentRel - currentAbs) > 0.02
                && Math.abs(targetAngle - getAngle()) < IntakeArmConstants.kTolerance) {
            syncCounter++;
            if (syncCounter > 10) {
                m_relativeEncoder.setPosition(currentAbs);
                syncCounter = 0;
            }
        } else {
            syncCounter = 0;
        }
    }

    /*
     * Boolean functions
     */
    private boolean isOpen() {
        return Math.abs(IntakeArmConstants.kOpenAngle - getAngle()) < IntakeArmConstants.kTolerance;
    }

    private boolean isClosed() {
        return Math.abs(IntakeArmConstants.kClosedAngle - getAngle()) < IntakeArmConstants.kTolerance;
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
            case PREPARING_SHOOTER:
                break;
            case HOME:
            case L1_CLOSE_AUTON:
            case L1_CLOSE_TELEOP:
            case L1_EXTEND_AUTON:
            case L1_EXTEND_TELEOP:
                CloseArm();
                break;
            case PREPARING_SHOOTER_AND_INTAKING:
            case SHOOTING_AND_INTAKING:
            case UNJAM:
            case INTAKING:
                OpenArm();
                break;
            case SHOOTING:
                handleArmShake();
                break;
        }
    }

    private void handleArmShake() {
        // Calculate how far out we should go this step
        targetDistance = IntakeArmConstants.kShakeMax - (IntakeArmConstants.KStepDistance * currentStep);

        MathUtil.clamp(targetDistance, IntakeArmConstants.kShakeMin, IntakeArmConstants.kShakeMax);

        // Cap it so we don't accidentally break the arm past kShakeMin
        if (targetDistance < IntakeArmConstants.kShakeMin) {
            targetDistance = IntakeArmConstants.kShakeMin;
        }

        if (isMovingOut) {
            setAngle(targetDistance);

            // Once we reach the outward step, flip direction to go back in
            if (getAngle() < targetDistance) {
                isMovingOut = false;
            }
            System.out.println("moving out step: " + currentStep);
        } else {
            setAngle(IntakeArmConstants.kShakeMax); // Retract back in

            // Once we fully retract, increment step and go back out
            if (getAngle() > IntakeArmConstants.kShakeMax) {
                isMovingOut = true;
                currentStep++;

                System.out.println("moving in step: " + currentStep);
            }
        }
    }

    private void tuning() {
        if (!DriverStation.isTest())
            return;

        // Status updates
        SmartDashboard.putNumber("Arm/Abs encoder angle", m_absoluteEncoder.get());
        SmartDashboard.putNumber("Arm/Rel encoder angle", getAngle());
        SmartDashboard.putNumber("Arm/Error", targetAngle - getAngle());
        SmartDashboard.putBoolean("Arm/Abs encoder is connected", m_absoluteEncoder.isConnected());
        SmartDashboard.putNumber("Arm/TargetAngle", targetAngle);
        // Read from dashboard
        double p = SmartDashboard.getNumber("Arm/kP", IntakeArmConstants.kP);
        double i = SmartDashboard.getNumber("Arm/kI", IntakeArmConstants.kI);
        double d = SmartDashboard.getNumber("Arm/kD", IntakeArmConstants.kD);
        double s = SmartDashboard.getNumber("Arm/kS", IntakeArmConstants.kS);
        double v = SmartDashboard.getNumber("Arm/kV", IntakeArmConstants.kV);
        double a = SmartDashboard.getNumber("Arm/kA", IntakeArmConstants.kA);
        double g = SmartDashboard.getNumber("Arm/kG", IntakeArmConstants.kG);
        double cosRatio = SmartDashboard.getNumber("Arm/kCosRatio", IntakeArmConstants.kCosRatio);

        double maxV = SmartDashboard.getNumber("Arm/Max Velocity", IntakeArmConstants.kCruiseVelocity);
        double maxA = SmartDashboard.getNumber("Arm/Max Accel", IntakeArmConstants.kMaxAcceleration);

        // Check if anything changed
        if (p != lastP || i != lastI || d != lastD || s != lastS ||
                v != lastV || a != lastA || g != lastG || cosRatio != lastCosRatio ||
                maxV != lastMaxVel || maxA != lastMaxAccel) {

            lastP = p;
            lastI = i;
            lastD = d;
            lastS = s;
            lastV = v;
            lastA = a;
            lastG = g;
            lastCosRatio = cosRatio;
            lastMaxVel = maxV;
            lastMaxAccel = maxA;

            SparkMaxConfig tuneConfig = new SparkMaxConfig();

            tuneConfig.closedLoop
                    .pid(p, i, d).feedForward.kS(s).kV(v).kA(a).kCos(g).kCosRatio(cosRatio);

            // Update the Trapezoidal Profile live
            tuneConfig.closedLoop.maxMotion
                    .cruiseVelocity(maxV)
                    .maxAcceleration(maxA);

            m_motor.configure(tuneConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("Arm: Tuning parameters updated! 🚀");
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
            case L1_CLOSE_AUTON:
            case L1_CLOSE_TELEOP:
            case L1_EXTEND_AUTON:
            case L1_EXTEND_TELEOP:
                return state == IntakeArmState.CLOSED;
            case INTAKING:
            case SHOOTING_AND_INTAKING:
            case PREPARING_SHOOTER_AND_INTAKING:
                return state == IntakeArmState.OPEN;
            case SHOOTING:
                return true;
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