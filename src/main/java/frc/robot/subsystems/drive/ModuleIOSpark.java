package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.driveEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveEncoderVelocityFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveKd;
import static frc.robot.subsystems.drive.DriveConstants.driveKp;
import static frc.robot.subsystems.drive.DriveConstants.driveKs;
import static frc.robot.subsystems.drive.DriveConstants.driveKv;
import static frc.robot.subsystems.drive.DriveConstants.driveMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMaxInput;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMinInput;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants.SwerveModuleConfig;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {

    // Hardware objects
    private final SparkFlex driveSpark;
    private final SparkFlex turnSpark;
    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    // Closed loop controllers
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController turnController;

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    public ModuleIOSpark(int module) {
        SwerveModuleConfig swerveConfig = DriveConstants.moduleConfigs[module];
        driveSpark = new SparkFlex(swerveConfig.driveCanId, MotorType.kBrushless);
        driveEncoder = driveSpark.getEncoder();
        turnSpark = new SparkFlex(swerveConfig.turnCanId, MotorType.kBrushless);
        turnEncoder = new CANcoder(swerveConfig.cancoderId);

        driveController = driveSpark.getClosedLoopController();
        turnController = turnSpark.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new SparkFlexConfig();
        driveConfig
                .inverted(swerveConfig.driveInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(driveMotorCurrentLimit)
                .voltageCompensation(12.0);
        driveConfig.encoder
                .positionConversionFactor(driveEncoderPositionFactor)
                .velocityConversionFactor(driveEncoderVelocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(driveKp, 0.0, driveKd);
        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                driveSpark,
                5,
                () -> driveSpark.configure(
                        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(swerveConfig.turnInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
                .voltageCompensation(12.0);
        turnConfig.encoder
                .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
                .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
                .pid(DriveConstants.turnKp, 0.0, DriveConstants.turnKd);
        turnConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        CANcoderConfiguration cancoderConf = new CANcoderConfiguration();

        cancoderConf.MagnetSensor.MagnetOffset = -swerveConfig.angleOffset.getRotations();
        cancoderConf.MagnetSensor.SensorDirection = swerveConfig.cancoderInverted
                ? SensorDirectionValue.CounterClockwise_Positive
                : SensorDirectionValue.Clockwise_Positive;

        turnEncoder.getConfigurator().apply(cancoderConf);
        turnSpark.getEncoder().setPosition(getAbsolutePosition());

        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, this::getAbsolutePosition);
    }

    @Override
    public void updateInputs(ModuleIOInputsAutoLogged inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
                driveSpark,
                new DoubleSupplier[] { driveSpark::getAppliedOutput, driveSpark::getBusVoltage },
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnSpark,
                this::getAbsolutePosition,
                (value) -> inputs.turnPosition = Rotation2d.fromRadians(value));
        ifOk(turnSpark, this::getAbsoluteVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveSpark.setVoltage(output);
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        double ffVolts = driveKs * Math.signum(velocityRadPerSec) + driveKv * velocityRadPerSec;
        driveController.setSetpoint(
                velocityRadPerSec,
                ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
                rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
        turnController.setSetpoint(setpoint, ControlType.kPosition);
    }

    public double getAbsolutePosition() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * DriveConstants.kCANcoderFactor;
    }

    public double getAbsoluteVelocity() {
        return turnEncoder.getVelocity().getValueAsDouble() * DriveConstants.kCANcoderFactor;
    }
}
