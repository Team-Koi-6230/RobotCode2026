package frc.robot.subsystems.shooter.flywheelIO;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryToSetPidfUntillOkTune;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.SparkPIDF;

public class ShooterIORev implements ShooterIO {
        private final SparkFlex m_spark, s_spark;

        private final SparkClosedLoopController controller;

        private double _targetRpm = 0;

        public ShooterIORev() {
                m_spark = new SparkFlex(RobotMap.CanBus.kMainShooterID, MotorType.kBrushless);
                s_spark = new SparkFlex(RobotMap.CanBus.kSecondaryShooterID, MotorType.kBrushless);

                SparkFlexConfig m_config = new SparkFlexConfig();
                m_config
                                .smartCurrentLimit(ShooterConstants.Flywheel.kShooterSmartCurrentLimit)
                                .idleMode(IdleMode.kCoast)
                                .voltageCompensation(12);

                m_config.signals
                                .primaryEncoderPositionAlwaysOn(true)
                                .primaryEncoderPositionPeriodMs(20)
                                .primaryEncoderVelocityAlwaysOn(true)
                                .primaryEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);

                m_config.closedLoop
                                .pid(ShooterConstants.Flywheel.kP, ShooterConstants.Flywheel.kI,
                                                ShooterConstants.Flywheel.kD).feedForward
                                .kS(ShooterConstants.Flywheel.kS)
                                .kV(ShooterConstants.Flywheel.kV)
                                .kA(ShooterConstants.Flywheel.kA);

                tryUntilOk(m_spark, 5,
                                () -> m_spark.configure(m_config, ResetMode.kNoResetSafeParameters,
                                                PersistMode.kPersistParameters));

                SparkFlexConfig s_config = new SparkFlexConfig();
                s_config.follow(m_spark, ShooterConstants.Flywheel.kFollowerInverted);

                tryUntilOk(m_spark, 5,
                                () -> s_spark.configure(s_config, ResetMode.kNoResetSafeParameters,
                                                PersistMode.kPersistParameters));

                controller = m_spark.getClosedLoopController();
        }

        @Override
        public void updateInputs(ShooterIOInputsAutoLogged inputs) {
                ifOk(m_spark, new DoubleSupplier[] { m_spark::getAppliedOutput, m_spark::getBusVoltage },
                                (vals) -> inputs.appliedVoltage = vals[0] * vals[1]);

                ifOk(m_spark, new DoubleSupplier[] { m_spark::getOutputCurrent, s_spark::getOutputCurrent },
                                (vals) -> inputs.current = new double[] {
                                                vals[0], vals[1] });

                inputs.targetRPM = _targetRpm;

                ifOk(m_spark, m_spark.getEncoder()::getVelocity, (rpm) -> inputs.currentRPM = rpm);
                ifOk(m_spark, m_spark.getEncoder()::getVelocity,
                                (rpm) -> inputs.radsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(rpm));
        }

        @Override
        public void runVolts(double volts) {
                volts = MathUtil.clamp(volts, -12, 12);
                _targetRpm = 0;
                controller.setSetpoint(volts, ControlType.kVoltage);
        }

        @Override
        public void stop() {
                _targetRpm = 0;
                controller.setSetpoint(0, ControlType.kVoltage);
        }

        @Override
        public void runRPM(double RPM) {
                _targetRpm = RPM;
                controller.setSetpoint(RPM, ControlType.kVelocity);
        }

        @Override
        public void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA) {
                SparkPIDF pidf = new SparkPIDF()
                                .withP(kP)
                                .withI(kI)
                                .withD(kD)
                                .withS(kS)
                                .withV(kV)
                                .withA(kA);

                tryToSetPidfUntillOkTune(m_spark, 5, pidf);
        }
}
