package frc.robot.subsystems.intake.io;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryToSetPidfUntillOkTune;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.SparkPIDF;

public class IntakeIORev implements IntakeIO {
        private SparkMax m_pivot;
        private RelativeEncoder m_relativeEncoder;
        private DutyCycleEncoder m_absoluteEncoder;
        private SparkClosedLoopController m_pivotController;

        private Rotation2d _targetAngle = IntakeConstants.kMaxAngle;

        private boolean _closedLoopPivot = false;

        public IntakeIORev() {
                m_pivot = new SparkMax(RobotMap.CanBus.kIntakePivotID, MotorType.kBrushless);

                m_relativeEncoder = m_pivot.getEncoder();

                m_absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.kIntakePivotThroughBoreID,
                                360 / IntakeConstants.kPivotShaftToPivotGearRatio,
                                IntakeConstants.kThroughBoreOffset);

                var pivotConfig = new SparkMaxConfig();
                pivotConfig
                                .inverted(IntakeConstants.kMotorInverted)
                                .idleMode(IdleMode.kBrake)
                                .voltageCompensation(12);
                pivotConfig
                                .smartCurrentLimit(IntakeConstants.kPivotCurrentLimits).softLimit
                                .forwardSoftLimit(IntakeConstants.kForwardSoftLimit)
                                .reverseSoftLimit(IntakeConstants.kReverseSoftLimit);

                pivotConfig.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs(20)
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);

                pivotConfig.closedLoop
                                .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD).feedForward
                                .kS(IntakeConstants.kS)
                                .kV(IntakeConstants.kV)
                                .kA(IntakeConstants.kA)
                                .kCos(IntakeConstants.kG)
                                .kCosRatio(IntakeConstants.kCosRatio);

                pivotConfig.closedLoop.maxMotion
                                .maxAcceleration(IntakeConstants.kMaxAcceleration)
                                .allowedProfileError(IntakeConstants.kTolerance)
                                .cruiseVelocity(IntakeConstants.kCruiseVelocity)
                                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

                pivotConfig.encoder
                                .inverted(IntakeConstants.kEncoderInverted)
                                .positionConversionFactor(360.0 / IntakeConstants.kPivotMotorToPivotGearRatio)
                                .velocityConversionFactor((360.0 / IntakeConstants.kPivotMotorToPivotGearRatio) / 60.0);

                tryUntilOk(
                                m_pivot,
                                5,
                                () -> m_pivot.configure(pivotConfig, ResetMode.kNoResetSafeParameters,
                                                PersistMode.kPersistParameters));

                m_pivotController = m_pivot.getClosedLoopController();

                tryUntilOk(m_pivot, 5, () -> m_relativeEncoder.setPosition(getAbsoluteEncoderDeg()));
        }

        @Override
        public void updateInputs(IntakeIOInputsAutoLogged inputs) {
                inputs.absolutePivotAngleDeg = getAbsoluteEncoderDeg();
                inputs.absolutePivotAngleRad = Math.toRadians(getAbsoluteEncoderDeg());
                ifOk(m_pivot, m_relativeEncoder::getPosition, (value) -> Math.abs(_targetAngle.getDegrees() - value));
                ifOk(
                                m_pivot,
                                new DoubleSupplier[] { m_pivot::getAppliedOutput, m_pivot::getBusVoltage },
                                (values) -> inputs.pivotAppliedVoltage = values[0] * values[1]);
                inputs.pivotClosedLoop = _closedLoopPivot;
                inputs.pivotCurrent = new double[] { m_pivot.getOutputCurrent() };
                ifOk(m_pivot, m_pivot::getOutputCurrent, (value) -> inputs.pivotCurrent = new double[] { value });
                inputs.pivotTargetAngle = _targetAngle.getDegrees();
                ifOk(m_pivot, m_relativeEncoder::getPosition, (value) -> inputs.relativePivotAngleDeg = value);
                ifOk(m_pivot, m_relativeEncoder::getPosition,
                                (value) -> inputs.relativePivotAngleRad = Math.toRadians(value));

        }

        @Override
        public void setTargetAngle(Rotation2d angle) {
                _closedLoopPivot = true;
                this._targetAngle = angle;
                m_pivotController.setSetpoint(angle.getDegrees(), ControlType.kMAXMotionPositionControl);
        }

        @Override
        public void runVoltsPivot(double volts) {
                _closedLoopPivot = false;
                m_pivotController.setSetpoint(volts, ControlType.kVoltage);
        }

        @Override
        public void stop() {
                m_pivot.stopMotor();
        }

        @Override
        public void setPIDF(double kP, double kI, double kD, double kS, double kG, double kV, double kA,
                        double kMaxVelocityRadPerSec, double kMaxAccelRadPerSecSquared) {
                SparkPIDF pidf = new SparkPIDF()
                                .withP(kP)
                                .withI(kI)
                                .withD(kD)
                                .withS(kS)
                                .withCos(kG, IntakeConstants.kCosRatio)
                                .withV(kV)
                                .withA(kA)
                                .withMAXMotion(kMaxVelocityRadPerSec, kMaxAccelRadPerSecSquared,
                                                IntakeConstants.kTolerance);
                tryToSetPidfUntillOkTune(m_pivot, 5, pidf);
        }

        private double getAbsoluteEncoderDeg() {
                var raw = m_absoluteEncoder.get();

                return IntakeConstants.kEncoderInverted ? (360 / IntakeConstants.kPivotShaftToPivotGearRatio - raw)
                                : raw;
        }

}
